#include "StdAfx.h"
#include "Stream.h"

#define PX_DELETE_ARRAY(x)	if (x) { delete []x;	x = NULL; }
//UserStream::UserStream(const char* filename, bool load) : fp(NULL)
//	{
//	fp = fopen(filename, load ? "rb" : "wb");
//	}
//
//UserStream::~UserStream()
//	{
//	if(fp)	fclose(fp);
//	}
//
//// Loading API
//PxU8 UserStream::readByte() const
//	{
//	PxU8 b;
//	size_t r = fread(&b, sizeof(PxU8), 1, fp);
//	PX_ASSERT(r);
//	return b;
//	}
//
//PxU16 UserStream::readWord() const
//	{
//	PxU16 w;
//	size_t r = fread(&w, sizeof(PxU16), 1, fp);
//	PX_ASSERT(r);
//	return w;
//	}
//
//PxU32 UserStream::readDword() const
//	{
//	PxU32 d;
//	size_t r = fread(&d, sizeof(PxU32), 1, fp);
//	PX_ASSERT(r);
//	return d;
//	}
//
//float UserStream::readFloat() const
//	{
//	PxReal f;
//	size_t r = fread(&f, sizeof(PxReal), 1, fp);
//	PX_ASSERT(r);
//	return f;
//	}
//
//double UserStream::readDouble() const
//	{
//	PxF64 f;
//	size_t r = fread(&f, sizeof(PxF64), 1, fp);
//	PX_ASSERT(r);
//	return f;
//	}
//
//void UserStream::readBuffer(void* buffer, PxU32 size)	const
//	{
//	size_t w = fread(buffer, size, 1, fp);
//	PX_ASSERT(w);
//	}
//
//// Saving API
//PxStream& UserStream::storeByte(PxU8 b)
//	{
//	size_t w = fwrite(&b, sizeof(PxU8), 1, fp);
//	PX_ASSERT(w);
//	return *this;
//	}
//
//PxStream& UserStream::storeWord(PxU16 w)
//	{
//	size_t ww = fwrite(&w, sizeof(PxU16), 1, fp);
//	PX_ASSERT(ww);
//	return *this;
//	}
//
//PxStream& UserStream::storeDword(PxU32 d)
//	{
//	size_t w = fwrite(&d, sizeof(PxU32), 1, fp);
//	PX_ASSERT(w);
//	return *this;
//	}
//
//PxStream& UserStream::storeFloat(PxReal f)
//	{
//	size_t w = fwrite(&f, sizeof(PxReal), 1, fp);
//	PX_ASSERT(w);
//	return *this;
//	}
//
//PxStream& UserStream::storeDouble(PxF64 f)
//	{
//	size_t w = fwrite(&f, sizeof(PxF64), 1, fp);
//	PX_ASSERT(w);
//	return *this;
//	}
//
//PxStream& UserStream::storeBuffer(const void* buffer, PxU32 size)
//	{
//	size_t w = fwrite(buffer, size, 1, fp);
//	PX_ASSERT(w);
//	return *this;
//	}
//
//
//

MemoryWriteBuffer::MemoryWriteBuffer() : currentSize(0), maxSize(0), data(NULL)
	{
	}




MemoryWriteBuffer::~MemoryWriteBuffer()
	{
	PX_DELETE_ARRAY(data);
	}

void MemoryWriteBuffer::clear()
	{
	currentSize = 0;
	}

PxOutputStream& MemoryWriteBuffer::storeByte(PxU8 b)
	{
	storeBuffer(&b, sizeof(PxU8));
	return *this;
	}
PxOutputStream& MemoryWriteBuffer::storeWord(PxU16 w)
	{
	storeBuffer(&w, sizeof(PxU16));
	return *this;
	}
PxOutputStream& MemoryWriteBuffer::storeDword(PxU32 d)
	{
	storeBuffer(&d, sizeof(PxU32));
	return *this;
	}
PxOutputStream& MemoryWriteBuffer::storeFloat(PxReal f)
	{
	storeBuffer(&f, sizeof(PxReal));
	return *this;
	}
PxOutputStream& MemoryWriteBuffer::storeDouble(PxF64 f)
	{
	storeBuffer(&f, sizeof(PxF64));
	return *this;
	}
PxOutputStream& MemoryWriteBuffer::storeBuffer(const void* buffer, PxU32 size)
	{
	PxU32 expectedSize = currentSize + size;
	if(expectedSize > maxSize)
		{
		maxSize = expectedSize + 4096;

		PxU8* newData = new PxU8[maxSize];
		PX_ASSERT(newData!=NULL);

		if(data)
			{
			memcpy(newData, data, currentSize);
			delete[] data;
			}
		data = newData;
		}
	memcpy(data+currentSize, buffer, size);
	currentSize += size;
	return *this;
	}

PxU32 MemoryWriteBuffer::write(const void* buffer, PxU32 size) 
{
	PxU32 expectedSize = currentSize + size;
	if(expectedSize > maxSize)
		{
		maxSize = expectedSize + 4096;

		PxU8* newData = new PxU8[maxSize];
		PX_ASSERT(newData!=NULL);

		if(data)
			{
			memcpy(newData, data, currentSize);
			delete[] data;
			}
		data = newData;
		}
	memcpy(data+currentSize, buffer, size);
	currentSize += size;
	return size;

}



MemoryReadBuffer::MemoryReadBuffer(const PxU8* data) : buffer(data)
	{
	}

MemoryReadBuffer::~MemoryReadBuffer()
	{
	// We don't own the data => no delete
	}

PxU8 MemoryReadBuffer::readByte() const
	{
	PxU8 b;
	memcpy(&b, buffer, sizeof(PxU8));
	buffer += sizeof(PxU8);
	return b;
	}

PxU16 MemoryReadBuffer::readWord() const
	{
	PxU16 w;
	memcpy(&w, buffer, sizeof(PxU16));
	buffer += sizeof(PxU16);
	return w;
	}

PxU32 MemoryReadBuffer::readDword() const
	{
	PxU32 d;
	memcpy(&d, buffer, sizeof(PxU32));
	buffer += sizeof(PxU32);
	return d;
	}

float MemoryReadBuffer::readFloat() const
	{
	float f;
	memcpy(&f, buffer, sizeof(float));
	buffer += sizeof(float);
	return f;
	}

double MemoryReadBuffer::readDouble() const
	{
	double f;
	memcpy(&f, buffer, sizeof(double));
	buffer += sizeof(double);
	return f;
	}

void MemoryReadBuffer::readBuffer(void* dest, PxU32 size) const
	{
	memcpy(dest, buffer, size);
	buffer += size;
	}


PxU32		MemoryReadBuffer::read(void* dest, PxU32 count)		
		{ 	
			
			memcpy(dest, buffer, count);
			buffer += count;
			return count;
		}