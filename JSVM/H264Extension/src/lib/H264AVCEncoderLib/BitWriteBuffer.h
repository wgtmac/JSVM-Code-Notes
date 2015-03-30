
#if !defined(AFX_BITWRITEBUFFER_H__F1308E37_7998_4953_9F78_FF6A3DBC22B5__INCLUDED_)
#define AFX_BITWRITEBUFFER_H__F1308E37_7998_4953_9F78_FF6A3DBC22B5__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "BitWriteBufferIf.h"

H264AVC_NAMESPACE_BEGIN

class BitWriteBuffer :
public BitWriteBufferIf
{
protected:
  BitWriteBuffer();
  virtual ~BitWriteBuffer();

public:

  BitWriteBufferIf* getNextBitWriteBuffer( Bool bStartNewBitstream );
  Bool nextBitWriteBufferActive() { return NULL != m_pucNextStreamPacket; }
  UChar* getNextBuffersPacket() { return m_pucNextStreamPacket; }

  static ErrVal create( BitWriteBuffer*& rpcBitWriteBuffer );
  ErrVal destroy();

  ErrVal init();
  ErrVal uninit() { return init(); }

  ErrVal initPacket( UInt* pulBits, UInt uiPacketLength );

  ErrVal write( UInt uiBits, UInt uiNumberOfBits = 1);

  UInt getNumberOfWrittenBits() { return  m_uiBitsWritten; }

  ErrVal pcmSamples( const TCoeff* pCoeff, UInt uiNumberOfSamples );

  ErrVal flushBuffer();
  ErrVal writeAlignZero();
  ErrVal writeAlignOne();

  ErrVal getLastByte(UChar &uiLastByte, UInt &uiLastBitPos);//FIX_FRAG_CAVLC
	//JVT-X046 {
	UInt   getDWordsLeft(void)  { return m_uiDWordsLeft;   }
	UInt   getBitsWritten(void) { return m_uiBitsWritten;  }
	Int    getValidBits(void)   { return m_iValidBits;     }
	UInt	 getCurrentBits(void) { return m_ulCurrentBits;  }
	UInt*  getStreamPacket(void){ return m_pulStreamPacket;}
	void loadBitWriteBuffer(BitWriteBufferIf* pcBitWriteBufferIf)
	{
		BitWriteBuffer* pcBitWriteBuffer = (BitWriteBuffer*)(pcBitWriteBufferIf);
		m_uiDWordsLeft = pcBitWriteBuffer->getDWordsLeft();
		m_uiBitsWritten = pcBitWriteBuffer->getBitsWritten();
		m_iValidBits = pcBitWriteBuffer->getValidBits();
		m_ulCurrentBits = pcBitWriteBuffer->getCurrentBits();
		m_pulStreamPacket = pcBitWriteBuffer->getStreamPacket();
	}
	void loadBitCounter(BitWriteBufferIf* pcBitWriteBufferIf){}
	UInt getBitsWriten() { return m_uiBitsWritten; }
	//JVT-X046 }
protected:
  UInt  xSwap( UInt ul )
  {
    // heiko.schwarz@hhi.fhg.de: support for BSD systems as proposed by Steffen Kamp [kamp@ient.rwth-aachen.de]
#ifdef MSYS_BIG_ENDIAN
    return ul;
#else
    UInt ul2;

    ul2  = ul>>24;
    ul2 |= (ul>>8) & 0x0000ff00;
    ul2 |= (ul<<8) & 0x00ff0000;
    ul2 |= ul<<24;

    return ul2;
#endif
  }

protected:
  UInt   m_uiDWordsLeft;       // =（ m_uiInitPacketLength+3）/ 4    剩余能写入的双字
  UInt   m_uiBitsWritten;                         //记录写了多少bit
  Int    m_iValidBits;                                 //1～32的可写空间
  UInt   m_ulCurrentBits;                       //当前写入的32比特
  UInt*  m_pulStreamPacket;                  //保存写的比特流

private:
  UInt   m_uiInitPacketLength;               //初始化开辟的缓存长度
  BitWriteBuffer * m_pcNextBitWriteBuffer;
  UChar *m_pucNextStreamPacket;
};



H264AVC_NAMESPACE_END


#endif // !defined(AFX_BITWRITEBUFFER_H__F1308E37_7998_4953_9F78_FF6A3DBC22B5__INCLUDED_)
