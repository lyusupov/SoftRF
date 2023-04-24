  // "hello world"
  Serial.println("5eb63bbbe01eeed093cb22bb8f5acdc3");
  unsigned char *hash = MD5::make_hash("hello world");
  char *md5str = MD5::make_digest(hash,16);
  Serial.println(md5str);
  free(hash);
  free(md5str); 