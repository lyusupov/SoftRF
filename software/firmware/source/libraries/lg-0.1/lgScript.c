/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
*/

#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <inttypes.h>

#include "lgpio.h"
#include "rgpiod.h"

#include "lgCmd.h"
#include "lgCtx.h"
#include "lgDbg.h"
#include "lgHdl.h"

#define LG_SCRIPT_HALT   0
#define LG_SCRIPT_RUN    1
#define LG_SCRIPT_DELETE 2

#define LG_SCRIPT_STACK_SIZE 256

typedef struct
{
   int id;
   int request;
   int run_state;
   pthread_t *pthIdp;
   pthread_mutex_t pthMutex;
   pthread_cond_t pthCond;
   cmdScript_t script;
   char user[LG_USER_LEN];
   int share;
} lgScript_t, *lgScript_p;


static void _scriptClose(lgScript_p s)
{
   LG_DBG(LG_DEBUG_ALWAYS, "objp=*%p", s);

   if (s->script.par) free(s->script.par);

   s->script.par = NULL;
}


int myScriptNameValid(char *name)
{
   int i, c, len, valid;

   len = strlen(name);

   valid = 1;

   for (i=0; i<len; i++)
   {
      c = name[i];

      if ((!isalnum(c)) && (c != '_') && (c != '-'))
      {
         valid = 0;
         break;
      }
   }
   return valid;
}

static int scrPop(lgScript_p s, int *SP, int *S)
{
   if ((*SP) > 0)
   {
      return S[--(*SP)];
   }
   else
   {
      s->run_state = LG_SCRIPT_FAILED;
      LG_DBG(LG_DEBUG_ALWAYS, "script %d too many pops", s->id);
      return 0;
   }
}

/* ----------------------------------------------------------------------- */

static void scrPush(lgScript_p s, int *SP, int *S, int val)
{
   if ((*SP) < LG_SCRIPT_STACK_SIZE)
   {
      S[(*SP)++] = val;
   }
   else
   {
      s->run_state = LG_SCRIPT_FAILED;
      LG_DBG(LG_DEBUG_ALWAYS, "script %d too many pushes", s->id);
   }
}

/* ----------------------------------------------------------------------- */

static void scrSwap(int *v1, int *v2)
{
   int t;

   t=*v1; *v1=*v2; *v2= t;
}

/* ----------------------------------------------------------------------- */

int scrSys(char *cmd, uint32_t p0, uint32_t p1)
{
   char cmdBuf[1024];
   int status;

   if (!myScriptNameValid(cmd))
      PARAM_ERROR(LG_BAD_SCRIPT_NAME, "bad script name (%s)", cmd);

   snprintf(cmdBuf, sizeof(cmdBuf), "%s/cgi/%s %u %u",
      lguGetWorkDir(), cmd, p0, p1);

   LG_DBG(LG_DEBUG_USER, "%s", cmdBuf);

   status = system(cmdBuf);

   if (status < 0) status = LG_BAD_SHELL_STATUS;

   return status;
}

static uint32_t xrl(uint32_t x, unsigned int n) 
{
   return (x << n) | (x >> (-n & 31));
}
  
static uint32_t xrr(uint32_t x, unsigned int n) 
{ 
   return (x >> n) | (x << (-n & 31));
}

static uint32_t xsl(uint32_t x, unsigned int n) 
{
   return (x << n);
}
  
static uint32_t xsr(uint32_t x, unsigned int n) 
{
   return (x >> n);
}

/* ----------------------------------------------------------------------- */

static void *pthScript(void *x)
{
   lgScript_p s;
   int i, t;
   cmdInstr_t instr;
   int32_t p0, p1, p0o, p1o, *t1, *t2;
   int32_t PC, A, F, SP;
   uint32_t tmp;
   int S[LG_SCRIPT_STACK_SIZE];
   lgCtx_p Ctx;
   lgCmd_t cmdBuf[CMD_MAX_EXTENSION/sizeof(lgCmd_t)];
   lgCmd_p cmdP=cmdBuf;
   uint32_t *arg=(uint32_t*)&cmdP[1];

   Ctx = lgCtxGet();

   if (!Ctx) return 0;

   s = x;

   strncpy(Ctx->user, s->user, LG_USER_LEN);
   Ctx->autoUseShare = s->share;

   s->run_state = LG_SCRIPT_READY;

   while ((volatile int)s->request != LG_SCRIPT_DELETE)
   {
      pthread_mutex_lock(&s->pthMutex);
      if ((volatile int)s->request != LG_SCRIPT_DELETE)
         pthread_cond_wait(&s->pthCond, &s->pthMutex);
      s->run_state = LG_SCRIPT_RUNNING;
      pthread_mutex_unlock(&s->pthMutex);

      A  = 0;
      F  = 0;
      PC = 0;
      SP = 0;

      while (((volatile int)s->request   == LG_SCRIPT_RUN    ) &&
                           (s->run_state == LG_SCRIPT_RUNNING))
      {
         // get local copy of next instruction

         instr = s->script.instr[PC];

         if (instr.cmd < LG_CMD_SCRIPT)
         {
            // parameter and variable substitution

            for (i=0; i<CMD_MAX_OPT; i++)
            {
               t = instr.arg[i];
               if      (instr.opt[i] == CMD_VAR) instr.arg[i] = s->script.var[t];
               else if (instr.opt[i] == CMD_PAR) instr.arg[i] = s->script.par[t];
            }

            fprintf(stderr, "PC=%d cmd=%d p0=%d p1=%d p2=%d p3=%d\n",
               PC, instr.cmd,
               instr.arg[0], instr.arg[1], instr.arg[2], instr.arg[3]);

            fflush(stderr);

            cmdP->magic = LG_MAGIC;
            cmdP->size = 0;
            cmdP->cmd = instr.cmd;
            cmdP->doubles = 0;
            cmdP->longs = 0;
            cmdP->shorts = 0;

            for (i=0; i<CMD_MAX_ARG; i++) arg[i] = instr.arg[i];

            A = lgExecCmd(cmdBuf, sizeof(cmdBuf));

            F = A;

            PC++;
         }
         else
         {
            p0o = instr.arg[0];
            p1o = instr.arg[1];

            // parameter and variable substitution

            for (i=0; i<2; i++) // maximum of two parameters
            {
               t = instr.arg[i];
               if      (instr.opt[i] == CMD_VAR) instr.arg[i] = s->script.var[t];
               else if (instr.opt[i] == CMD_PAR) instr.arg[i] = s->script.par[t];
            }

            p0 = instr.arg[0];
            p1 = instr.arg[1];

            fprintf(stderr, "PC=%d cmd=%d p0=%d p0o=%d p1=%d p1o=%d\n",
               PC, instr.cmd,
               instr.arg[0], p0o, instr.arg[1], p1o);

            fflush(stderr);

            switch (instr.cmd)
            {
               case LG_CMD_ADD:   A+=p0; F=A;                     PC++; break;

               case LG_CMD_AND:   A&=p0; F=A;                     PC++; break;

               case LG_CMD_CALL:  scrPush(s, &SP, S, PC+1);    PC = p0; break;

               case LG_CMD_CMP:   F=A-p0;                         PC++; break;

               case LG_CMD_DCR:
                  if (instr.opt[0] == CMD_PAR)
                     {--s->script.par[p0o]; F=s->script.par[p0o];}
                  else
                     {--s->script.var[p0o]; F=s->script.var[p0o];}
                  PC++;
                  break;

               case LG_CMD_DCRA:  --A; F=A;                       PC++; break;

               case LG_CMD_DIV:   A/=p0; F=A;                     PC++; break;

               case LG_CMD_HALT:  s->run_state = LG_SCRIPT_ENDED;       break;

               case LG_CMD_INR:
                  if (instr.opt[0] == CMD_PAR)
                     {++s->script.par[p0o]; F=s->script.par[p0o];}
                  else
                     {++s->script.var[p0o]; F=s->script.var[p0o];}
                  PC++;
                  break;

               case LG_CMD_INRA:  ++A; F=A;                       PC++; break;

               case LG_CMD_JLE:   if (F<=0) PC=p0; else PC++;           break;

               case LG_CMD_JLT:   if (F<0)  PC=p0; else PC++;           break;

               case LG_CMD_JMP:   PC=p0;                                break;

               case LG_CMD_JNZ:   if (F)    PC=p0; else PC++;           break;

               case LG_CMD_JGE:   if (F>=0) PC=p0; else PC++;           break;

               case LG_CMD_JGT:   if (F>0)  PC=p0; else PC++;           break;

               case LG_CMD_JZ:    if (!F)   PC=p0; else PC++;           break;

               case LG_CMD_LD:
                  if (instr.opt[0] == CMD_PAR) s->script.par[p0o]=p1;
                  else                         s->script.var[p0o]=p1;
                  PC++;
                  break;

               case LG_CMD_LDA:   A=p0;                           PC++; break;

               case LG_CMD_MLT:   A*=p0; F=A;                     PC++; break;

               case LG_CMD_MOD:   A%=p0; F=A;                     PC++; break;

               case LG_CMD_OR:    A|=p0; F=A;                     PC++; break;

               case LG_CMD_POP:
                  if (instr.opt[0] == CMD_PAR)
                     s->script.par[p0o]=scrPop(s, &SP, S);
                  else
                     s->script.var[p0o]=scrPop(s, &SP, S);
                  PC++;
                  break;

               case LG_CMD_POPA:  A=scrPop(s, &SP, S);            PC++; break;

               case LG_CMD_PUSH:
                  if (instr.opt[0] == CMD_PAR)
                     scrPush(s, &SP, S, s->script.par[p0o]);
                  else
                     scrPush(s, &SP, S, s->script.var[p0o]);
                  PC++;
                  break;

               case LG_CMD_PUSHA: scrPush(s, &SP, S, A);          PC++; break;

               case LG_CMD_RET:   PC=scrPop(s, &SP, S);                 break;

               case LG_CMD_RL:
                  if (instr.opt[0] == CMD_PAR)
                  {
                     tmp = xrl(s->script.par[p0o], p1);
                     s->script.par[p0o] = tmp;
                     F=tmp;
                  }
                  else
                  {
                     tmp = xrl(s->script.var[p0o], p1);
                     s->script.var[p0o] = tmp;
                     F=tmp;
                  }
                  PC++;
                  break;

               case LG_CMD_RLA:   A=xrl(A, p0); F=A;              PC++; break;

               case LG_CMD_RR:
                  if (instr.opt[0] == CMD_PAR)
                  {
                     tmp = xrr(s->script.par[p0o], p1);
                     s->script.par[p0o] = tmp;
                     F=tmp;
                  }
                  else
                  {
                     tmp = xrr(s->script.var[p0o], p1);
                     s->script.var[p0o] = tmp;
                     F=tmp;
                  }
                  PC++;
                  break;

               case LG_CMD_RRA:   A=xrr(A, p0); F=A;              PC++; break;

               case LG_CMD_SHL:
                  if (instr.opt[0] == CMD_PAR)
                  {
                     tmp = xsl(s->script.par[p0o], p1);
                     s->script.par[p0o] = tmp;
                     F=tmp;
                  }
                  else
                  {
                     tmp = xsl(s->script.var[p0o], p1);
                     s->script.var[p0o] = tmp;
                     F=tmp;
                  }
                  PC++;
                  break;

               case LG_CMD_SHLA:   A=xsl(A, p0); F=A;              PC++; break;

               case LG_CMD_SHR:
                  if (instr.opt[0] == CMD_PAR)
                  {
                     tmp = xsr(s->script.par[p0o], p1);
                     s->script.par[p0o] = tmp;
                     F=tmp;
                  }
                  else
                  {
                     tmp = xsr(s->script.var[p0o], p1);
                     s->script.var[p0o] = tmp;
                     F=tmp;
                  }
                  PC++;
                  break;

               case LG_CMD_SHRA:   A=xsr(A, p0); F=A;              PC++; break;

               case LG_CMD_STA:
                  if (instr.opt[0] == CMD_PAR) s->script.par[p0o]=A;
                  else                         s->script.var[p0o]=A;
                  PC++;
                  break;

               case LG_CMD_SUB:   A-=p0; F=A;                     PC++; break;

               case LG_CMD_SYS:
                  //A=scrSys((char*)instr.arg[4], A, 0);
                  F=A;
                  PC++;
                  break;

               case LG_CMD_X:
                  if (instr.opt[0] == CMD_PAR) t1 = &s->script.par[p0o];
                  else                         t1 = &s->script.var[p0o];

                  if (instr.opt[1] == CMD_PAR) t2 = &s->script.par[p1o];
                  else                         t2 = &s->script.var[p1o];

                  scrSwap(t1, t2);
                  PC++;
                  break;

               case LG_CMD_XA:
                  if (instr.opt[0] == CMD_PAR)
                     scrSwap(&s->script.par[p0o], &A);
                  else
                     scrSwap(&s->script.var[p0o], &A);
                  PC++;
                  break;

               case LG_CMD_XOR:   A^=p0; F=A;                     PC++; break;

            }
         }

         if (PC >= s->script.instrs) s->run_state = LG_SCRIPT_ENDED;
      }

      if (((volatile int)s->request == LG_SCRIPT_HALT)  ||
          ((volatile int)s->request == LG_SCRIPT_DELETE))
         s->run_state = LG_SCRIPT_HALTED;
   }

   lgHdlPurgeByOwner(Ctx->owner);

   LG_DBG(LG_DEBUG_ALWAYS, "free context memory %d", Ctx->owner);

   free(Ctx);

   s->run_state = LG_SCRIPT_EXITED;  

   return 0;
}

/* ----------------------------------------------------------------------- */

void lgRawDumpScript(int handle)
{
   int i;
   lgScript_p s;
   int status;
   
   LG_DBG(LG_DEBUG_USER, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SCRIPT, (void **)&s);

   if (status == LG_OKAY)
   {
      for (i=0; i<LG_MAX_SCRIPT_PARAMS; i++)
      {
         fprintf(stderr, "p%d=%d ", i, s->script.par[i]);
      }

      fprintf(stderr, "\n");

      for (i=0; i<LG_MAX_SCRIPT_VARS; i++)
      {
         fprintf(stderr, "v%d=%d ", i, s->script.var[i]);
      }

      fprintf(stderr, "\n");

      for (i=0; i<s->script.instrs; i++)
      {
         fprintf(stderr, 
            "%d: cmd=%d [%d(%d), %d(%d), %d(%d), %d(%d)]\n",
            i,
            s->script.instr[i].cmd,
            s->script.instr[i].arg[0],
            s->script.instr[i].opt[0],
            s->script.instr[i].arg[1],
            s->script.instr[i].opt[1],
            s->script.instr[i].arg[2],
            s->script.instr[i].opt[2],
            s->script.instr[i].arg[3],
            s->script.instr[i].opt[3]);
      }

      lgHdlUnlock(handle);
   }
}

int lgScriptStore(char *script)
{
   lgScript_p s;
   lgCtx_p Ctx;
   int handle;
   int status;

   LG_DBG(LG_DEBUG_TRACE, "script=[%s]", script);

   handle = lgHdlAlloc(
      LG_HDL_TYPE_SCRIPT, sizeof(lgScript_t), (void**)&s, _scriptClose);

   if (handle < 0) return LG_NO_MEMORY;

   status = cmdParseScript(script, &s->script, 0);

   if (status == 0)
   {
      /* set the owner's user and share */

      Ctx = lgCtxGet();

      if (Ctx)
      {
         strncpy(s->user, Ctx->user, LG_USER_LEN);
         s->share = Ctx->autoUseShare;
      }

      s->request   = LG_SCRIPT_HALT;
      s->run_state = LG_SCRIPT_INITING;

      pthread_cond_init(&s->pthCond, NULL);
      pthread_mutex_init(&s->pthMutex, NULL);

      s->id = handle;

      s->pthIdp = lgThreadStart(pthScript, s);

      status = handle;
   }
   else lgHdlFree(handle, LG_HDL_TYPE_SCRIPT);

   return status;
}


/* ----------------------------------------------------------------------- */

int lgScriptRun(int handle, int count, uint32_t *scriptParam)
{
   int status=0;
   int i;
   lgScript_p s;
   
   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d scriptParam=%08"PRIXPTR,
      handle, count, (uintptr_t)scriptParam);

   if ((unsigned)count > LG_MAX_SCRIPT_PARAMS)
      PARAM_ERROR(LG_TOO_MANY_PARAM, "bad number of parameters(%d)", count);
      
   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SCRIPT, (void **)&s);

   if (status == LG_OKAY)
   {
      pthread_mutex_lock(&s->pthMutex);

      if (s->run_state != LG_SCRIPT_INITING)
      {
         if (scriptParam != NULL)
         {
            for (i=0; i<count; i++) s->script.par[i] = scriptParam[i];
         }

         s->request = LG_SCRIPT_RUN;

         pthread_cond_signal(&s->pthCond);
      }
      else
      {
         status = LG_SCRIPT_NOT_READY;
      }

      pthread_mutex_unlock(&s->pthMutex);

      lgHdlUnlock(handle);
   }

   return status;
}


/* ----------------------------------------------------------------------- */

int lgScriptUpdate(int handle, int count, uint32_t *scriptParam)
{
   int status;
   int i;
   lgScript_p s;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d count=%d scriptParam=%08"PRIXPTR,
      handle, count, (uintptr_t)scriptParam);

   if ((unsigned)count > LG_MAX_SCRIPT_PARAMS)
      PARAM_ERROR(LG_TOO_MANY_PARAM, "bad number of parameters(%d)", count);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SCRIPT, (void **)&s);

   if (status == LG_OKAY)
   {
      if (scriptParam != NULL)
      {
         pthread_mutex_lock(&s->pthMutex);

         for (i=0; i<count; i++) s->script.par[i] = scriptParam[i];

         pthread_mutex_unlock(&s->pthMutex);
      }

      lgHdlUnlock(handle);
   }

   return status;
}


/* ----------------------------------------------------------------------- */

int lgScriptStatus(int handle, uint32_t *scriptParam)
{
   int status;
   int i;
   lgScript_p s;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d scriptParam=%08"PRIXPTR,
      handle, (uintptr_t)scriptParam);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SCRIPT, (void **)&s);

   if (status == LG_OKAY)
   {
      pthread_mutex_lock(&s->pthMutex);

      if (scriptParam != NULL)
      {
         for (i=0; i<LG_MAX_SCRIPT_PARAMS; i++)
            scriptParam[i] = s->script.par[i];
      }

      status = s->run_state;

      if (status >= LG_SCRIPT_ENDED) s->run_state = LG_SCRIPT_READY;

      pthread_mutex_unlock(&s->pthMutex);

      lgHdlUnlock(handle);
   }

   return status;
}


/* ----------------------------------------------------------------------- */

int lgScriptStop(int handle)
{
   int status;
   lgScript_p s;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SCRIPT, (void **)&s);

   if (status == LG_OKAY)
   {
      pthread_mutex_lock(&s->pthMutex);

      s->request = LG_SCRIPT_HALT;

      pthread_cond_signal(&s->pthCond);

      pthread_mutex_unlock(&s->pthMutex);

      lgHdlUnlock(handle);
   }

   return status;
}

/* ----------------------------------------------------------------------- */

int lgScriptDelete(int handle)
{
   int status;
   lgScript_p s;
   pthread_t *pthIdp;

   LG_DBG(LG_DEBUG_TRACE, "handle=%d", handle);

   status = lgHdlGetLockedObj(handle, LG_HDL_TYPE_SCRIPT, (void **)&s);

   if (status == LG_OKAY)
   {
      pthread_mutex_lock(&s->pthMutex);

      s->request = LG_SCRIPT_DELETE;

      pthread_cond_signal(&s->pthCond);

      pthread_mutex_unlock(&s->pthMutex);

      while (s->run_state != LG_SCRIPT_EXITED)
      {
         usleep(5000); /* give script time to halt */
      }

      pthIdp = s->pthIdp;

      status = lgHdlFree(handle, LG_HDL_TYPE_SCRIPT);

      lgHdlUnlock(handle);

      lgThreadStop(pthIdp);
   }

   return status;
}


int lgShell(char *scriptName, char *scriptString)
{
   int status;
   char cmdBuf[4096];

   LG_DBG(LG_DEBUG_TRACE, "name=%s string=%s", scriptName, scriptString);

   if (!myScriptNameValid(scriptName))
      PARAM_ERROR(LG_BAD_SCRIPT_NAME, "bad script name (%s)", scriptName);

   snprintf(cmdBuf, sizeof(cmdBuf),
      "%s/cgi/%s %s", lguGetConfigDir(), scriptName, scriptString);

   LG_DBG(LG_DEBUG_USER, "%s", cmdBuf);

   status = system(cmdBuf);

   if (status < 0) status = LG_BAD_SHELL_STATUS;

   return status;
}

