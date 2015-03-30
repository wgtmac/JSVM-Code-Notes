
#include "H264AVCEncoderLib.h"
#include "BitWriteBuffer.h"
#include "NalUnitEncoder.h"

#include "CodingParameter.h"

H264AVC_NAMESPACE_BEGIN


NalUnitEncoder::NalUnitEncoder()
: m_bIsUnitActive         ( false )
, m_pcBitWriteBuffer      ( 0 )
, m_pcHeaderSymbolWriteIf ( 0 )
, m_pcHeaderSymbolTestIf  ( 0 )
, m_pcBinDataAccessor     ( 0 )
, m_pucBuffer             ( 0 )
, m_pucTempBuffer         ( 0 )
, m_pucTempBufferBackup   ( 0 )
, m_uiPacketLength        ( MSYS_UINT_MAX )
, m_eNalUnitType          ( NAL_UNIT_UNSPECIFIED_0 )
, m_eNalRefIdc            ( NAL_REF_IDC_PRIORITY_LOWEST )
{
}


NalUnitEncoder::~NalUnitEncoder()
{
}


ErrVal
NalUnitEncoder::create( NalUnitEncoder*& rpcNalUnitEncoder )
{
  rpcNalUnitEncoder = new NalUnitEncoder;
  ROT( NULL == rpcNalUnitEncoder );
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::init( BitWriteBuffer*       pcBitWriteBuffer,
                      HeaderSymbolWriteIf*  pcHeaderSymbolWriteIf,
                      HeaderSymbolWriteIf*  pcHeaderSymbolTestIf )
{
  ROT( NULL == pcBitWriteBuffer );
  ROT( NULL == pcHeaderSymbolWriteIf );
  ROT( NULL == pcHeaderSymbolTestIf );

  m_pcBitWriteBuffer      = pcBitWriteBuffer;
  m_pcHeaderSymbolTestIf  = pcHeaderSymbolTestIf;
  m_pcHeaderSymbolWriteIf = pcHeaderSymbolWriteIf;
  m_bIsUnitActive         = false;
  m_pucBuffer             = NULL;
  m_pucTempBuffer         = NULL;
  m_pucTempBufferBackup   = NULL;
  m_uiPacketLength        = MSYS_UINT_MAX;
  m_eNalUnitType          = NAL_UNIT_UNSPECIFIED_0;
  m_eNalRefIdc            = NAL_REF_IDC_PRIORITY_LOWEST;

  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::uninit()
{
  delete [] m_pucTempBuffer;
  delete [] m_pucTempBufferBackup;
  m_pucTempBufferBackup   = NULL;

  m_pcBitWriteBuffer      = NULL;
  m_pcHeaderSymbolWriteIf = NULL;
  m_pcHeaderSymbolTestIf  = NULL;
  m_bIsUnitActive         = false;
  m_pucBuffer             = NULL;
  m_pucTempBuffer         = NULL;
  m_uiPacketLength        = MSYS_UINT_MAX;
  m_eNalUnitType          = NAL_UNIT_UNSPECIFIED_0;
  m_eNalRefIdc            = NAL_REF_IDC_PRIORITY_LOWEST;

  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::destroy()
{
  uninit();
  delete this;
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::initNalUnit( BinDataAccessor* pcBinDataAccessor )
{
  ROT( m_bIsUnitActive );
  ROF( pcBinDataAccessor );
  ROT( pcBinDataAccessor->size() < 1 );

  m_bIsUnitActive     = true;
  m_pcBinDataAccessor = pcBinDataAccessor;                //NalUnitEncoder 类和 LayerEncoder 的 BinDataAccessor 合二为一
  m_pucBuffer         = pcBinDataAccessor->data();

  if( m_uiPacketLength != m_pcBinDataAccessor->size() )               //之前开辟的空间不够用
  {
    delete [] m_pucTempBuffer;

    m_uiPacketLength = m_pcBinDataAccessor->size();                  //开辟的长度
    m_pucTempBuffer  = new UChar[ m_uiPacketLength ];             //临时buffer
    ROF( m_pucTempBuffer );
  }
                               //BitWriteBuffer 的 m_pulStreamPacket 与 NalUnitEncoder 的 m_pucTempBuffer 联系起来了
  RNOK( m_pcBitWriteBuffer->initPacket( (UInt*)(m_pucTempBuffer), m_uiPacketLength-1 ) );             //初始化packet  用来对齐8比特/32比特啥的

  return Err::m_nOK;
}



ErrVal
NalUnitEncoder::closeAndAppendNalUnits( UInt                    *pauiBits,
                                        ExtBinDataAccessorList  &rcExtBinDataAccessorList,
                                        ExtBinDataAccessor      *pcExtBinDataAccessor,
                                        BinData                 &rcBinData,
                                        H264AVCEncoder          *pcH264AVCEncoder,
                                        UInt                    uiQualityLevelCGSSNR,
                                        UInt                    uiLayerCGSSNR )
{
  ROF( m_bIsUnitActive );
  ROF( pcExtBinDataAccessor );
  ROF( pcExtBinDataAccessor->data() );

  ROF( m_pcBinDataAccessor == pcExtBinDataAccessor );

  //===== write trailing bits =====
  if( NAL_UNIT_END_OF_SEQUENCE != m_eNalUnitType &&
      NAL_UNIT_END_OF_STREAM   != m_eNalUnitType &&
     (NAL_UNIT_PREFIX          != m_eNalUnitType || m_eNalRefIdc != NAL_REF_IDC_PRIORITY_LOWEST) )
  {
    RNOK( xWriteTrailingBits() );
  }
  RNOK( m_pcBitWriteBuffer->flushBuffer() );

  //===== convert to payload and add header =====
  UInt  uiHeaderBytes = 1;
  if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_SCALABLE || m_eNalUnitType == NAL_UNIT_PREFIX )
  {
    uiHeaderBytes += NAL_UNIT_HEADER_SVC_EXTENSION_BYTES;
  }

  BitWriteBufferIf *pcCurrentWriteBuffer = m_pcBitWriteBuffer;       // 写 packet
  UChar            *pucPayload           = m_pucBuffer;                              //pucPayload 就是写入NalUnit的数据 现在没内容
  const UChar      *pucRBSP              = m_pucTempBuffer;                  // RBSP 当前比特流
  UInt              uiPayloadBufferSize  = m_uiPacketLength;

  ROF( pcExtBinDataAccessor->data() == pucPayload          );
  ROF( pcExtBinDataAccessor->size() == uiPayloadBufferSize );

  UInt uiFragment = 0;
  while( true )
  {
    UInt uiBits  = pcCurrentWriteBuffer->getNumberOfWrittenBits();
    UInt uiBytes = ( uiBits + 7 ) >> 3;
    RNOK( convertRBSPToPayload( uiBytes, uiHeaderBytes, pucPayload, pucRBSP, uiPayloadBufferSize ) ); 
    pauiBits[uiFragment] = 8 * uiBytes;

    UChar* pucNewBuffer = new UChar [ uiBytes ];
    ROF( pucNewBuffer );
    memcpy( pucNewBuffer, pucPayload, uiBytes * sizeof( UChar ) );     //PayLoad 转移到  pucNewBuffer

    if( pcH264AVCEncoder )            //   暂时没看  SEI   ***********************************START*************************************
    {
      //JVT-W052
      if(pcH264AVCEncoder->getCodingParameter()->getIntegrityCheckSEIEnable() && pcH264AVCEncoder->getCodingParameter()->getCGSSNRRefinement() )
      {
        if( uiQualityLevelCGSSNR + uiFragment > 0 )
        {
          UInt uicrcMsb,uicrcVal;
          uicrcVal = pcH264AVCEncoder->m_uicrcVal[uiLayerCGSSNR];
          uicrcMsb = 0;
          Bool BitVal = false;
          for ( UInt uiBitIdx = 0; uiBitIdx< uiBytes*8; uiBitIdx++ )
          {
            uicrcMsb = ( uicrcVal >> 15 ) & 1;
            BitVal = ( pucNewBuffer[uiBitIdx>>3] >> (7-(uiBitIdx&7)) )&1;
            uicrcVal = (((uicrcVal<<1) + BitVal ) & 0xffff)^(uicrcMsb*0x1021);
          }
          pcH264AVCEncoder->m_uicrcVal[uiLayerCGSSNR] = uicrcVal;
          if( pcH264AVCEncoder->m_uiNumofCGS[uiLayerCGSSNR] == uiQualityLevelCGSSNR + uiFragment )
          {
            ROT( pcCurrentWriteBuffer->nextBitWriteBufferActive() );
            for(UInt uiBitIdx = 0; uiBitIdx< 16; uiBitIdx++)
            {
              uicrcMsb = ( uicrcVal >> 15 ) & 1;
              BitVal = 0;
              uicrcVal = (((uicrcVal<<1) + BitVal ) & 0xffff)^(uicrcMsb*0x1021);
            }
            pcH264AVCEncoder->m_uicrcVal[uiLayerCGSSNR] = uicrcVal;
            UInt uiInfoEntry = pcH264AVCEncoder->m_pcIntegrityCheckSEI->getNumInfoEntriesMinus1();
            if( uiInfoEntry == MSYS_UINT_MAX )
            {
              uiInfoEntry = 0;
            }
            else
            {
              UInt uiLastLayer = pcH264AVCEncoder->m_pcIntegrityCheckSEI->getEntryDependencyId(uiInfoEntry);
              if( uiLastLayer < uiLayerCGSSNR )
              {
                uiInfoEntry++;
              }
            }
            pcH264AVCEncoder->m_pcIntegrityCheckSEI->setNumInfoEntriesMinus1(uiInfoEntry);
            pcH264AVCEncoder->m_pcIntegrityCheckSEI->setEntryDependencyId(uiInfoEntry,uiLayerCGSSNR);
            pcH264AVCEncoder->m_pcIntegrityCheckSEI->setQualityLayerCRC(uiInfoEntry,uicrcVal);
          }
        }
      }
      //JVT-W052
    }                             //   暂时没看  SEI   ***********************************END*************************************

    ExtBinDataAccessor* pcNewExtBinDataAccessor = new ExtBinDataAccessor;
    ROF( pcNewExtBinDataAccessor );

    rcBinData               .reset          ();
    rcBinData               .set            (  pucNewBuffer, uiBytes );              //  Payload 的内容
    rcBinData               .setMemAccessor ( *pcNewExtBinDataAccessor );      // 新的 pcNewExtBinDataAccessor 来操纵当前内容
    rcExtBinDataAccessorList.push_back      (  pcNewExtBinDataAccessor );

    rcBinData               .reset          ();
    rcBinData               .setMemAccessor ( *pcExtBinDataAccessor );     // 老的 pcExtBinDataAccessor 仍然操作以前内容 Payload 仍在

    if( !pcCurrentWriteBuffer->nextBitWriteBufferActive() )
    {
      break;
    }
    pucRBSP              = pcCurrentWriteBuffer->getNextBuffersPacket();
    pcCurrentWriteBuffer = pcCurrentWriteBuffer->getNextBitWriteBuffer();
    uiFragment++;               //有几个 MGS 质量层
  }

  RNOK( m_pcBitWriteBuffer->uninit() );

  //==== reset parameters =====
  m_bIsUnitActive     = false;
  m_pucBuffer         = NULL;
  m_pcBinDataAccessor = NULL;
  m_eNalUnitType      = NAL_UNIT_UNSPECIFIED_0;
  m_eNalRefIdc        = NAL_REF_IDC_PRIORITY_LOWEST;
  return Err::m_nOK;
}



ErrVal
NalUnitEncoder::closeNalUnit( UInt& ruiBits )
{
  ROF( m_bIsUnitActive );

  //===== write trailing bits =====
  if( NAL_UNIT_END_OF_SEQUENCE != m_eNalUnitType &&
      NAL_UNIT_END_OF_STREAM   != m_eNalUnitType &&
     (NAL_UNIT_PREFIX          != m_eNalUnitType || m_eNalRefIdc != NAL_REF_IDC_PRIORITY_LOWEST) )   // m_eNalRefIdc != NAL_REF_IDC_PRIORITY_LOWEST就不可能含NAL_UNIT_PREFIX
  {
    RNOK ( xWriteTrailingBits() );      //写100.。。补齐
  }
  RNOK( m_pcBitWriteBuffer->flushBuffer() );              //把拖尾等最后的数据写入并计算bit

  //===== convert to payload and add header =====
  UInt  uiHeaderBytes = 1;
  if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_SCALABLE || m_eNalUnitType == NAL_UNIT_PREFIX )
  {
    uiHeaderBytes += NAL_UNIT_HEADER_SVC_EXTENSION_BYTES;
  }

  UInt  uiBits = ( m_pcBitWriteBuffer->getNumberOfWrittenBits() + 7 ) >> 3;          //加0x03前的Bytes

  RNOK( convertRBSPToPayload( uiBits, uiHeaderBytes, m_pucBuffer, m_pucTempBuffer, m_uiPacketLength ) );          //加上0x03
  RNOK( m_pcBinDataAccessor->decreaseEndPos( m_pcBinDataAccessor->size() - uiBits ) );      //这一步干什么用--------记录使用的byte数 移动末尾 使没用到的空间无效
  ruiBits             = 8*uiBits;        //加0x03后的bytes

  //==== reset parameters =====
  m_bIsUnitActive     = false;
  m_pucBuffer         = NULL;                  //数据已经写完  不需要操纵了  与LayerEncoder 的 m_cBinData关联的内容 脱离关系
  m_pcBinDataAccessor = NULL;          //数据已经写完  不需要操纵了  与LayerEncoder 的 BinDataAccessor 脱离关系
  m_eNalUnitType      = NAL_UNIT_UNSPECIFIED_0;
  m_eNalRefIdc        = NAL_REF_IDC_PRIORITY_LOWEST;
  return Err::m_nOK;
}

ErrVal
NalUnitEncoder::convertRBSPToPayload( UInt         &ruiBytesWritten,
                                       UInt          uiHeaderBytes,
                                       UChar        *pcPayload,
                                       const UChar  *pcRBSP,
                                       UInt          uiPayloadBufferSize )
{
  UInt uiZeroCount    = 0;
  UInt uiReadOffset   = uiHeaderBytes;
  UInt uiWriteOffset  = uiHeaderBytes;

  //===== NAL unit header =====
  for( UInt uiIndex = 0; uiIndex < uiHeaderBytes; uiIndex++ )
  {
    pcPayload[uiIndex] = pcRBSP[uiIndex];
  }

  //===== NAL unit payload =====
  for( ; uiReadOffset < ruiBytesWritten ; uiReadOffset++, uiWriteOffset++ )
  {
    ROT( uiWriteOffset >= uiPayloadBufferSize );

    if( 2 == uiZeroCount && 0 == ( pcRBSP[uiReadOffset] & 0xfc ) )
    {
      uiZeroCount                   = 0;
      pcPayload[uiWriteOffset++]  = 0x03;
    }

    pcPayload[uiWriteOffset] = pcRBSP[uiReadOffset];

    if( 0 == pcRBSP[uiReadOffset] )
    {
      uiZeroCount++;
    }
    else
    {
      uiZeroCount = 0;
    }
  }
  if( ( 0x00 == pcPayload[uiWriteOffset-1] ) && ( 0x00 == pcPayload[uiWriteOffset-2] ) )
  {
    ROT( uiWriteOffset >= uiPayloadBufferSize );
    pcPayload[uiWriteOffset++] = 0x03;
  }
  ruiBytesWritten = uiWriteOffset;

  return Err::m_nOK;
}

ErrVal NalUnitEncoder::xWriteTrailingBits( )
{
  RNOK( m_pcBitWriteBuffer->write( 1 ) );
  RNOK( m_pcBitWriteBuffer->writeAlignZero() );    //补齐1000.。。到一个字节

  BitWriteBufferIf* pcCurrentBitWriter = m_pcBitWriteBuffer;
  while( pcCurrentBitWriter->nextBitWriteBufferActive() )             //把链接下去所有的结点都写上trailing bits
  {
    pcCurrentBitWriter = pcCurrentBitWriter->getNextBitWriteBuffer();
    RNOK( pcCurrentBitWriter->write( 1 ) );
    RNOK( pcCurrentBitWriter->writeAlignZero() );
  }
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::write( const SequenceParameterSet& rcSPS )
{
  RNOK( rcSPS.write( m_pcHeaderSymbolWriteIf ) );
  m_eNalUnitType  = rcSPS.getNalUnitType();
  m_eNalRefIdc    = NAL_REF_IDC_PRIORITY_HIGHEST;
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::write( const PictureParameterSet& rcPPS )
{
  RNOK( rcPPS.write( m_pcHeaderSymbolWriteIf ) );

  m_eNalUnitType  = rcPPS.getNalUnitType();
  m_eNalRefIdc    = NAL_REF_IDC_PRIORITY_HIGHEST;
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::writePrefix( const SliceHeader& rcSH )
{
  RNOK( rcSH.writePrefix( *m_pcHeaderSymbolWriteIf ) );            //写到了m_pcHeaderSymbolWriteIf变量中
  m_eNalUnitType  = NAL_UNIT_PREFIX;
  m_eNalRefIdc    = rcSH.getNalRefIdc();
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::write( const SliceHeader& rcSH )
{
  SliceHeader           cSH           = rcSH;
  HeaderSymbolWriteIf*  pcCurrWriteIf = m_pcHeaderSymbolWriteIf;    //NalUnitEncoder 的 HeaderSymbolWriteIf m_pcHeaderSymbolWriteIf用来写比特流
                                                                                                                           //而 NalUnitEncoder  的   BitWriteBuffer*       m_pcBitWriteBuffer 将 RBSP转为 PayLoad  （可以发送出去的）
  for( UInt uiMGSFragment = 0; true; uiMGSFragment++ )               //MGS SNR层
  {
    //----- modify copy of slice header -----
    cSH.setDependencyId                   ( rcSH.getLayerCGSSNR         () );
    cSH.setQualityId                      ( rcSH.getQualityLevelCGSSNR  () + uiMGSFragment );
    cSH.setDiscardableFlag                ( rcSH.getDiscardableFlag     () );
    cSH.setNoInterLayerPredFlag           ( rcSH.getNoInterLayerPredFlag() && cSH.getQualityId() == 0 );
    cSH.setScanIdxStart                   ( rcSH.getSPS().getMGSCoeffStart( uiMGSFragment ) );
    cSH.setScanIdxStop                    ( rcSH.getSPS().getMGSCoeffStop ( uiMGSFragment ) );
    cSH.setRefLayerDQId                   ( uiMGSFragment == 0 ? rcSH.getRefLayerDQId                   () : ( rcSH.getLayerCGSSNR() << 4 ) + rcSH.getQualityLevelCGSSNR() + uiMGSFragment - 1 );
    cSH.setAdaptiveBaseModeFlag           ( uiMGSFragment == 0 ? rcSH.getAdaptiveBaseModeFlag           () : false  );
    cSH.setAdaptiveMotionPredictionFlag   ( uiMGSFragment == 0 ? rcSH.getAdaptiveMotionPredictionFlag   () : false  );
    cSH.setAdaptiveResidualPredictionFlag ( uiMGSFragment == 0 ? rcSH.getAdaptiveResidualPredictionFlag () : false  );
    cSH.setDefaultBaseModeFlag            ( uiMGSFragment == 0 ? rcSH.getDefaultBaseModeFlag            () : true   );
    cSH.setDefaultMotionPredictionFlag    ( uiMGSFragment == 0 ? rcSH.getDefaultMotionPredictionFlag    () : true   );
    cSH.setDefaultResidualPredictionFlag  ( uiMGSFragment == 0 ? rcSH.getDefaultResidualPredictionFlag  () : true   );

    //----- write copy of slice header -----
    RNOK( cSH.write( *pcCurrWriteIf ) );
    if( rcSH.getSPS().getMGSCoeffStop( uiMGSFragment ) == 16 )
    {
      break;
    }

    //----- update -----
    pcCurrWriteIf = pcCurrWriteIf->getHeaderSymbolWriteIfNextSlice( true );
  }

  m_eNalUnitType  = rcSH.getNalUnitType ();
  m_eNalRefIdc    = rcSH.getNalRefIdc   ();
  return Err::m_nOK;
}


ErrVal
NalUnitEncoder::write( SEI::MessageList& rcSEIMessageList )
{
  RNOK( SEI::write( m_pcHeaderSymbolWriteIf, m_pcHeaderSymbolTestIf, &rcSEIMessageList ) );

  m_eNalUnitType  = NAL_UNIT_SEI;
  m_eNalRefIdc    = NAL_REF_IDC_PRIORITY_LOWEST;
  return Err::m_nOK;
}


// JVT-V068 {
ErrVal
NalUnitEncoder::writeScalableNestingSei( SEI::MessageList& rcSEIMessageList )
{
  RNOK( SEI::writeScalableNestingSei( m_pcHeaderSymbolWriteIf, m_pcHeaderSymbolTestIf, &rcSEIMessageList ) );
  m_eNalUnitType  = NAL_UNIT_SEI;
  m_eNalRefIdc    = NAL_REF_IDC_PRIORITY_LOWEST;
  return Err::m_nOK;
}
// JVT-V068 }

H264AVC_NAMESPACE_END
