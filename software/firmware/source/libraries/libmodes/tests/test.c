#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <assert.h>
#include "mode-s.h"

#define MODE_S_DATA_LEN (16*16384) // 256k
#define MODE_S_PREAMBLE_US 8       // microseconds
#define MODE_S_LONG_MSG_BITS 112
#define MODE_S_FULL_LEN (MODE_S_PREAMBLE_US+MODE_S_LONG_MSG_BITS)

#define MODE_S_NOTUSED(V) ((void) V)

pthread_t reader_thread;
pthread_mutex_t data_mutex; // Mutex to synchronize buffer access.
pthread_cond_t data_cond;   // Conditional variable associated.
unsigned char *data;        // Raw IQ samples buffer
int fd;                     // file descriptor.
uint32_t data_len;          // Buffer length.
int data_ready = 0;         // Data ready to be processed.
int should_exit = 0;        // Exit from the main loop when true.

void read_data_from_file(void) {
  pthread_mutex_lock(&data_mutex);
  while(1) {
    ssize_t nread, toread;
    unsigned char *p;

    if (data_ready) {
      pthread_cond_wait(&data_cond, &data_mutex);
      continue;
    }

    // Move the last part of the previous buffer, that was not processed, on
    // the start of the new buffer.
    memcpy(data, data+MODE_S_DATA_LEN, (MODE_S_FULL_LEN-1)*4);
    toread = MODE_S_DATA_LEN;
    p = data+(MODE_S_FULL_LEN-1)*4;
    while(toread) {
      nread = read(fd, p, toread);
      if (nread <= 0) {
        should_exit = 1; // Signal the other thread to exit.
        break;
      }
      p += nread;
      toread -= nread;
    }
    if (toread) {
      // Not enough data on file to fill the buffer? Pad with no signal.
      memset(p, 127, toread);
    }
    data_ready = 1;
    // Signal to the other thread that new data is ready
    pthread_cond_signal(&data_cond);
  }
}

// We read data using a thread, so the main thread only handles decoding
// without caring about data acquisition.
void *reader_thread_entry_point(void *arg) {
  MODE_S_NOTUSED(arg);
  read_data_from_file();
  return NULL;
}

int msgNo = 0;
char *messages[8] = {
  "8d45ac2d9904d910613f94ba81b5",
  "5d45ac2da5e9cb",
  "8d45ac2d583561285c4fa686fcdc",
  "a00006979b580030400000df4221",
  "5d45ac2da5e9cb",
  "5d45ac2da5e9cb",
  "02a186b39408d0",
  "200006b31828c8"
};

void test(mode_s_t *self, struct mode_s_msg *mm) {
  MODE_S_NOTUSED(self);
  int j;

  char *msg = (char*)malloc(mm->msgbits/4 * sizeof(char) + 1);
  for (j = 0; j < mm->msgbits/8; j++) sprintf(&msg[j*2], "%02x", mm->msg[j]);
  printf("validating message #%d\n", msgNo + 1);

  assert(strcmp(msg, messages[msgNo++]) == 0);
}

int main(int argc, char **argv) {
  mode_s_t state;
  uint16_t *mag;

  if (argc != 2) {
    fprintf(stderr, "Provide data filename as first argument\n");
    exit(1);
  }

  char *filename = strdup(argv[1]);
  if ((fd = open(filename, O_RDONLY)) == -1) {
    perror("Opening data file");
    exit(1);
  }

  data_len = MODE_S_DATA_LEN + (MODE_S_FULL_LEN-1)*4;
  if ((data = malloc(data_len)) == NULL ||
    (mag = malloc(sizeof(uint16_t) * (data_len / 2))) == NULL) {
    fprintf(stderr, "Out of memory allocating data buffer.\n");
    exit(1);
  }

  mode_s_init(&state);

  pthread_create(&reader_thread, NULL, reader_thread_entry_point, NULL);

  pthread_mutex_lock(&data_mutex);
  while(1) {
    if (!data_ready) {
      pthread_cond_wait(&data_cond, &data_mutex);
      continue;
    }
    mode_s_compute_magnitude_vector(data, mag, data_len);

    // Signal to the other thread that we processed the available data and we
    // want more.
    data_ready = 0;
    pthread_cond_signal(&data_cond);

    // Process data after releasing the lock, so that the capturing thread can
    // read data while we perform computationally expensive stuff * at the same
    // time. (This should only be useful with very slow processors).
    pthread_mutex_unlock(&data_mutex);
    mode_s_detect(&state, mag, data_len/2, test);
    pthread_mutex_lock(&data_mutex);
    if (should_exit) break;
  }

  printf("all ok\n");
  return 0;
}
