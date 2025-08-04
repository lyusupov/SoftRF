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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "lgCfg.h"

#define CFG_COMMENT    '#'
#define CFG_KEYVAL_SEP '='
#define CFG_SSECT      '['
#define CFG_ESECT      ']'

#define CFG_MAGIC 3705902252

typedef struct lgCfgKV_s lgCfgKV_t, *lgCfgKV_p;

typedef struct lgCfgKV_s
{
   char *name;
   char *value;
   lgCfgKV_p next_key;
} lgCfgKV_t, *lgCfgKV_p;

typedef struct lgCfgS_s lgCfgS_t, *lgCfgS_p;

typedef struct lgCfgS_s
{
   char *name;
   lgCfgS_p next_section;
   lgCfgKV_p first_key;
} lgCfgS_t, *lgCfgS_p;

typedef struct lgCfg_s
{
   uint32_t magic;
   char *file;
   lgCfgS_p first_section;
} lgCfg_t, *lgCfg_p;

lgCfg_p lgCfgNew(char *file)
{
   lgCfg_p cfg;

   cfg = calloc(1, sizeof(lgCfg_t));

   if (cfg == NULL) return NULL;

   cfg->file = strdup(file);

   if (cfg->file == NULL)
   {
      free(cfg);
      return NULL;
   }

   cfg->first_section = calloc(1, sizeof(lgCfgS_t));

   if (cfg->first_section == NULL)
   {
      free(cfg->file);
      free(cfg);
      return NULL;
   }

   cfg->first_section->name = strdup("global");

   if (cfg->first_section->name == NULL)
   {
      free(cfg->first_section);
      free(cfg->file);
      free(cfg);
      return NULL;
   }

   cfg->first_section->next_section = NULL;

   cfg->first_section->first_key = NULL;

   cfg->magic = CFG_MAGIC;

   return cfg;
}

lgCfgKV_p lgCfgFindKey(lgCfgS_p section, char *key)
{
   lgCfgKV_p k;

   for (k = section->first_key; k; k = k->next_key)
   {
      if (!strcmp(k->name, key)) break;
   }
   return k;
}

lgCfgS_p lgCfgFindSection(lgCfg_p cfg, char *section)
{
   lgCfgS_p s;

   for (s = cfg->first_section; s; s = s->next_section)
   {
      if (!strcmp(s->name, section)) break;
   }
   return s;
}

char *lgCfgGetValue(lgCfg_p cfg, char *section, char *key)
{
   lgCfgS_p cfgS;
   lgCfgKV_p cfgKV;

   cfgS = lgCfgFindSection(cfg, section);
   if (cfgS != NULL)
   {
      cfgKV = lgCfgFindKey(cfgS, key);
      if (cfgKV != NULL) return cfgKV->value;
   }
   return NULL;
 }

static lgCfgS_p lgCfgAddSection(lgCfg_p cfg, char *section)
{
   lgCfgS_p s;

   s = lgCfgFindSection(cfg, section);

   if (s == NULL)
   {
      s = calloc(1, sizeof(lgCfgS_t));

      if (s == NULL) return NULL;

      s->name = strdup(section);

      if (s->name == NULL) {free(s); return NULL;}

      s->first_key = NULL;

      s->next_section = cfg->first_section;

      cfg->first_section = s;
   }

   return s;
}

lgCfgKV_p lgCfgAddKeyValue(lgCfg_p cfg, lgCfgS_p section, char *key, char *value)
{
   lgCfgKV_p kv;

   kv = calloc(1, sizeof(lgCfgKV_t));

   if (kv == NULL) return NULL;

   kv->name = strdup(key);

   if (kv->name == NULL) {free(kv); return NULL;}

   kv->value = strdup(value);

   if (kv->value == NULL) {free(kv->name); free(kv); return NULL;}

   kv->next_key = section->first_key;

   section->first_key = kv;

   return kv;
}

static int lgCfgReadSection(lgCfg_p cfg, char *p, char **section)
{
   char *q, *r;

   *section = NULL;

   ++p;

   while (*p && isspace(*p)) ++p;

   for (q = p;
       *q && (*q != '\r') && (*q != '\n') && (*q != CFG_ESECT);
       ++q) ;

   if (*q != CFG_ESECT)
   {
      return CFG_BAD_FILE;
   }

   r = q + 1;

   while (*q && (q > p) && isspace(*(q - 1))) --q;

   if (q == p) return CFG_BAD_FILE;

   *q = 0;
   *section = p;

   while (*r && isspace(*r)) ++r;

   if (*r && (*r != CFG_COMMENT)) return CFG_BAD_FILE;

   return CFG_OKAY;
}

static int lgCfgReadKeyValue(lgCfg_p cfg, char *p, char **key, char **val)
{
   char *q, *v;

   *key = NULL;
   *val = NULL;

   while (*p && isspace(*p)) ++p;

   for (q = p;
       *q && (*q != '\r') && (*q != '\n') && (*q != CFG_KEYVAL_SEP);
       ++q) ;

   if (*q != CFG_KEYVAL_SEP) return CFG_BAD_FILE;

   v = q + 1;

   while (*q && (q > p) && isspace(*(q - 1))) --q;

   if (q == p) /* no key name */ return CFG_BAD_FILE;

   *q = 0;

   *key = p;

   while (*v && isspace(*v)) ++v;

   for (q = v;
       *q && (*q != '\r') && (*q != '\n') && (*q != CFG_COMMENT);
       ++q) ;

   while (*q && (q > v) && isspace(*(q - 1))) --q;

   if (q == v) /* no value */ return CFG_MISSING_VALUE;

   *q = 0;
   *val = v;

   return CFG_OKAY;
}

void lgCfgFree(lgCfg_p cfg)
{
   lgCfgS_p  s, s_next;
   lgCfgKV_p kv, kv_next;

   if ((cfg == NULL) || (cfg->magic != CFG_MAGIC)) return;

   s = cfg->first_section;

   while (s)
   {
      s_next = s->next_section;

      kv = s->first_key;

      while (kv)
      {
         kv_next = kv->next_key;
         free(kv->name);
         free(kv->value);
         free(kv);
         kv = kv_next;
      }

      free(s->name);
      free(s);
      s = s_next;
   }

   free(cfg->file);
   free(cfg);
}

lgCfg_p lgCfgRead(char *file)
{
   FILE *fp;
   lgCfgS_p cs = NULL; /* current section */
   char          *p       = NULL;
   char          *section = "";
   char          *key     = NULL;
   char          *val     = NULL;

   lgCfg_p cfg = NULL;
   int err = CFG_OKAY;
   char buf[2048];
   int bufPos;
   int done;

   fp = fopen(file, "r");

   if (fp == NULL)
   {
      return NULL;
   }

   cfg = lgCfgNew(file);

   if (cfg == NULL)
   {
      fclose(fp);
      return NULL;
   }

   cs = cfg->first_section;

   while (!feof(fp))
   {
      bufPos = 0;

      done = 0;

      while (!done)
      {
         if (fgets(buf+bufPos, sizeof(buf)-bufPos, fp) != NULL)
         {
            bufPos = strlen(buf);

            if ((bufPos < 2)             ||
                (buf[bufPos-1] != '\n')  ||
                (buf[bufPos-2] != '\\')  ||
                (bufPos >= (sizeof(buf)-2)))
               done = 1;
            else bufPos -= 2;
         }
         else done = 1;
      }

      if (!bufPos) continue;

      for (p = buf; *p && isspace(*p); ++p) ;

      if (!*p || (*p == CFG_COMMENT)) continue;

      if (*p == CFG_SSECT)
      {
         err = lgCfgReadSection(cfg, p, &section);
         if (err != CFG_OKAY)
         {
            lgCfgFree(cfg);
            cfg = NULL;
            break;
         }

         cs = lgCfgAddSection(cfg, section);
         if (cs == NULL)
         {
            lgCfgFree(cfg);
            cfg = NULL;
            break;
         }
      }
      else
      {
         err = lgCfgReadKeyValue(cfg, p, &key, &val);
         if (err != CFG_OKAY)
         {
            lgCfgFree(cfg);
            cfg = NULL;
            break;
         }

         if (lgCfgAddKeyValue(cfg, cs, key, val) == NULL)
         {
            lgCfgFree(cfg);
            cfg = NULL;
            break;
         }
      }
   }

   fclose(fp);

   return cfg;
}

void lgCfgPrint(lgCfg_p cfg, FILE *stream)
{
   lgCfgS_p  s;
   lgCfgKV_p kv;

   if ((cfg == NULL) || (cfg->magic != CFG_MAGIC)) return;

   for (s = cfg->first_section; s; s = s->next_section)
   {
      if (s->name) fprintf(stream, "[%s]\n", s->name);

      for (kv = s->first_key; kv; kv = kv->next_key)

         fprintf(stream, "%s=%s\n", kv->name, kv->value);

      fprintf(stream, "\n");
   }
}

char *lgCfgStrip(char *str)
{
  char *end;

  while(isspace((unsigned char)*str)) str++;

  if(*str == 0) return str;

  end = str + strlen(str) - 1;
  while (end > str && isspace((unsigned char)*end)) end--;

  end[1] = '\0';

  return str;
}

char *lgCfgNextToken(char **str, const char *delim, char **pos)
{
   char *token;

   if (*str) *pos = NULL;

   token = strtok_r(*str, delim, pos);
   *str = NULL;

   if (token) return lgCfgStrip(token);

   return NULL;
}

