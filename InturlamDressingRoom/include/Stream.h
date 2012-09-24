#ifndef STREAM_H
#define STREAM_H

#include "PxIO.h"

using namespace physx;

//class UserStream : public PxStream
//	{
//	public:
//								UserStream(const char* filename, bool load);
//	virtual						~UserStream();
//
//	virtual		PxU8			readByte()								const;
//	virtual		PxU16			readWord()								const;
//	virtual		PxU32			readDword()								const;
//	virtual		float			readFloat()								const;
//	virtual		double			readDouble()							const;
//	virtual		void			readBuffer(void* buffer, PxU32 size)	const;
//
//	virtual		PxStream&		storeByte(PxU8 b);
//	virtual		PxStream&		storeWord(PxU16 w);
//	virtual		PxStream&		storeDword(PxU32 d);
//	virtual		PxStream&		storeFloat(PxReal f);
//	virtual		PxStream&		storeDouble(PxF64 f);
//	virtual		PxStream&		storeBuffer(const void* buffer, PxU32 size);
//
//				FILE*			fp;
//	};

class MemoryWriteBuffer : public PxOutputStream
	{
	public:
								MemoryWriteBuffer();
	virtual						~MemoryWriteBuffer();
				void			clear();

				
	virtual		PxU8			readByte()								const	{ PX_ASSERT(0);	return 0;	}
	virtual		PxU16			readWord()								const	{ PX_ASSERT(0);	return 0;	}
	virtual		PxU32			readDword()								const	{ PX_ASSERT(0);	return 0;	}
	virtual		float			readFloat()								const	{ PX_ASSERT(0);	return 0.0f;}
	virtual		double			readDouble()							const	{ PX_ASSERT(0);	return 0.0;	}
	virtual		void			readBuffer(void* buffer, PxU32 size)	const	{ PX_ASSERT(0);				}

	virtual		PxOutputStream&		storeByte(PxU8 b);
	virtual		PxOutputStream&		storeWord(PxU16 w);
	virtual		PxOutputStream&		storeDword(PxU32 d);
	virtual		PxOutputStream&		storeFloat(PxReal f);
	virtual		PxOutputStream&		storeDouble(PxF64 f);
	virtual		PxOutputStream&		storeBuffer(const void* buffer, PxU32 size);
				PxU32 write(const void* buffer, PxU32 size) ;
				PxU32			currentSize;
				PxU32			maxSize;
				PxU8*			data;
	};

class MemoryReadBuffer : public PxInputStream
	{
	public:
								MemoryReadBuffer(const PxU8* data);
	virtual						~MemoryReadBuffer();

	virtual		PxU8			readByte()								const;
	virtual		PxU16			readWord()								const;
	virtual		PxU32			readDword()								const;
	virtual		float			readFloat()								const;
	virtual		double			readDouble()							const;
	virtual		void			readBuffer(void* buffer, PxU32 size)	const;
				PxU32			read(void* dest, PxU32 count)		;


	virtual		PxInputStream&		storeByte(PxU8 b)							{ PX_ASSERT(0);	return *this;	}
	virtual		PxInputStream&		storeWord(PxU16 w)							{ PX_ASSERT(0);	return *this;	}
	virtual		PxInputStream&		storeDword(PxU32 d)							{ PX_ASSERT(0);	return *this;	}
	virtual		PxInputStream&		storeFloat(PxReal f)						{ PX_ASSERT(0);	return *this;	}
	virtual		PxInputStream&		storeDouble(PxF64 f)						{ PX_ASSERT(0);	return *this;	}
	virtual		PxInputStream&		storeBuffer(const void* buffer, PxU32 size)	{ PX_ASSERT(0);	return *this;	}
	

	mutable		const PxU8*		buffer;
	};

#endif
