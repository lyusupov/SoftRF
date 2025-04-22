// RHEncryptedDriver.cpp
//
// Author: Philippe.Rochat'at'gmail.com
// Contributed to the RadioHead project by the author
// $Id: RHEncryptedDriver.cpp,v 1.7 2020/08/04 09:02:14 mikem Exp $

#include <RadioHead.h>
#ifdef RH_ENABLE_ENCRYPTION_MODULE
#include <RHEncryptedDriver.h>

RHEncryptedDriver::RHEncryptedDriver(RHGenericDriver& driver, BlockCipher& blockcipher)
    : _driver(driver),
      _blockcipher(blockcipher)
{
    _buffer = (uint8_t *)calloc(_driver.maxMessageLength(), sizeof(uint8_t));
}

bool RHEncryptedDriver::recv(uint8_t* buf, uint8_t* len)
{
    int h = 0; // Index of output _buffer

    bool status = _driver.recv(_buffer, len);
    if (status && buf && len)
    {
	int blockSize = _blockcipher.blockSize(); // Size of blocks used by encryption
	int nbBlocks = *len / blockSize; 	  // Number of blocks in that message
	if (nbBlocks * blockSize == *len)
	{
	    // Or we have a missmatch ... this is probably not symetrically encrypted 
	    for (int k = 0; k < nbBlocks; k++)
	    {
		// Decrypt each block
		_blockcipher.decryptBlock(&buf[h], &_buffer[k*blockSize]); // Decrypt that block into buf	
		h += blockSize;
#ifdef STRICT_CONTENT_LEN	
		if (k == 0)
		{
//		    if (buf[0] > *len - 1)
//			return false; // Bogus payload length
		    *len = buf[0]; // First byte contains length
		    h--;	   // First block is of length--
		    memmove(buf, buf+1, blockSize - 1);
		}
#endif			
	    }
	}
    }

    return status;
}

bool RHEncryptedDriver::send(const uint8_t* data, uint8_t len)
{
    if (len > maxMessageLength())
	return false;
    
    bool status = true;
    int blockSize = _blockcipher.blockSize(); // Size of blocks used by encryption
	
    if (len == 0) // PassThru
	return _driver.send(data, len);

    if (_cipheringBlocks.blockSize != blockSize)
    {
	// Cipher has changed it's block size
	_cipheringBlocks.inputBlock = (uint8_t *)realloc(_cipheringBlocks.inputBlock, blockSize);
	_cipheringBlocks.blockSize = blockSize;	
    }
	
    int max_message_length = maxMessageLength();
#ifdef STRICT_CONTENT_LEN	
    uint8_t nbBlocks = len / blockSize + 1; // How many blocks do we need for that message
    uint8_t nbBpM = (max_message_length + 1) / blockSize; // Max number of blocks per message
#else
    uint8_t nbBlocks = (len - 1) / blockSize + 1; // How many blocks do we need for that message
    uint8_t nbBpM = max_message_length / blockSize; // Max number of blocks per message
#endif	
    int k = 0, j = 0; // k is block index, j is original message index
#ifndef ALLOW_MULTIPLE_MSG	
#ifdef STRICT_CONTENT_LEN
    for (k = 0; k < nbBpM && k * blockSize < len + 1; k++)
#else
    for (k = 0; k < nbBpM && k * blockSize < len; k++)
#endif
    {
	// k blocks in that message
	int h = 0; // h is block content index
#ifdef STRICT_CONTENT_LEN
	if (k == 0)
	    _cipheringBlocks.inputBlock[h++] = len; // put in first byte of first block the message length
#endif		
	while (h < blockSize)
	{	
	    // Copy each msg byte into inputBlock, and trail with 0 if necessary
	    if (j < len)
		_cipheringBlocks.inputBlock[h++] = data[j++];
	    else
		_cipheringBlocks.inputBlock[h++] = 0; // Completing with trailing 0
	}
	_blockcipher.encryptBlock(&_buffer[k * blockSize], _cipheringBlocks.inputBlock); // Cipher that message into _buffer
    }
//    Serial.println(max_message_length);
//    Serial.println(nbBlocks);
//    Serial.println(nbBpM);
//    Serial.println(k);
//    Serial.println(blockSize);
//    printBuffer("single send", _buffer, k * blockSize);
    if (!_driver.send(_buffer, k*blockSize))  // We now send that message with it's new length
	status = false;
#else	
    uint8_t nbMsg = (nbBlocks * blockSize) / max_message_length + 1; // How many message do we need

    for (int i = 0; i < nbMsg; i++)
    {
	// i is message index
	for (k = 0; k < nbBpM && k * blockSize < len ; k++)
	{
	    // k blocks in that message
	    int h = 0;
#ifdef STRICT_CONTENT_LEN
	    if (k == 0 && i == 0)
		_cipheringBlocks.inputBlock[h++] = len; // put in first byte of first block of first message the message length
#endif			
	    while (h < blockSize)
	    {		
		// Copy each msg byte into inputBlock, and trail with 0 if necessary
		if (j < len)
		    _cipheringBlocks.inputBlock[h++] = data[j++];
		else
		    _cipheringBlocks.inputBlock[h++] = 0;
	    }
	    _blockcipher.encryptBlock(&_buffer[k * blockSize], _cipheringBlocks.inputBlock); // Cipher that message into buffer
	}
//	printBuffer("multiple send", _buffer, k * blockSize);
	if (!_driver.send(_buffer, k * blockSize))  // We now send that message with it's new length
	    status = false;
    }
#endif
    return status;
}

uint8_t RHEncryptedDriver::maxMessageLength()
{
    int driver_len = _driver.maxMessageLength();
    
#ifndef ALLOW_MULTIPLE_MSG
    driver_len = ((int)(driver_len/_blockcipher.blockSize()) ) * _blockcipher.blockSize();
#endif

#ifdef STRICT_CONTENT_LEN
    driver_len--;
#endif
    return driver_len;
}

#endif
