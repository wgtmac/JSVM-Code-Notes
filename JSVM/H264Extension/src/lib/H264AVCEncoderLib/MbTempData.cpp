
#include "H264AVCEncoderLib.h"

#include "MbTempData.h"

#include "H264AVCCommonLib/MbData.h"
#include "H264AVCCommonLib/YuvMbBuffer.h"



H264AVC_NAMESPACE_BEGIN


ErrVal IntMbTempData::init( MbDataAccess& rcMbDataAccess )
{                                                   //如果指针不存在，则开辟内存 并用后面的构造函数初始化  否则就用原指针 用placement new去初始化
  m_pcMbDataAccess = new( m_pcMbDataAccess ) MbDataAccess( rcMbDataAccess, *this );     //重载了operator new()  ，用 rcMbDataAccess 和 当前 this构造的内容初始化 m_pcMbDataAccess
  clear();
  return Err::m_nOK;
}

ErrVal IntMbTempData::uninit()
{
  return Err::m_nOK;
}


IntMbTempData::IntMbTempData() :
m_pcMbDataAccess( NULL )
{
  m_pcMbDataAccess = NULL;
  clear();

  MbData::init( this, &m_acMbMvdData[0], &m_acMbMvdData[1], &m_acMbMotionData[0], &m_acMbMotionData[1] );
}


IntMbTempData::~IntMbTempData()
{
  delete m_pcMbDataAccess;
  m_pcMbDataAccess = NULL;
}


Void IntMbTempData::clear()       //把父类所有变量(继承而来的)清空
{
  MbData::clear();
  YuvMbBuffer::setZero();           // 清除 m_aucYuvBuffer

  MbDataStruct::clear();
  CostData::clear();
  MbTransformCoeffs::clear();          //重置变换量化后和反量化反变换后系数
}



Void IntMbTempData::clearCost()
{
  CostData::clear();
}



Void IntMbTempData::copyTo( MbDataAccess& rcMbDataAccess )
{
  rcMbDataAccess.getMbData()            .copyFrom( *this );      //把当前 IntMbTempData的 MbDataStruct结构数据 => rcMbDataAccess 的 MbData部分
  rcMbDataAccess.getMbTCoeffs()         .copyFrom( *this );    //把当前 IntMbTempData的 MbTransformCoeffs结构数据 => rcMbDataAccess 的 MbTransformCoeffs部分

  rcMbDataAccess.getMbMvdData(LIST_0)   .copyFrom( m_acMbMvdData[LIST_0] );       //拷贝 Motion 和 MV 信息
  rcMbDataAccess.getMbMotionData(LIST_0).copyFrom( m_acMbMotionData[LIST_0] );

  if( rcMbDataAccess.getSH().isBSlice() )
  {
    rcMbDataAccess.getMbMvdData(LIST_1)   .copyFrom( m_acMbMvdData[LIST_1] );
    rcMbDataAccess.getMbMotionData(LIST_1).copyFrom( m_acMbMotionData[LIST_1] );
  }
}


Void IntMbTempData::copyResidualDataTo( MbDataAccess& rcMbDataAccess )
{
  rcMbDataAccess.getMbData    ().setBCBP              ( getBCBP             () );
  rcMbDataAccess.getMbData    ().setMbExtCbp          ( getMbExtCbp         () );
  rcMbDataAccess.getMbData    ().setQp                ( getQp               () );
  rcMbDataAccess.getMbData    ().setQp4LF             ( getQp4LF            () );
  rcMbDataAccess.getMbTCoeffs ().copyFrom             ( *this                  );
  rcMbDataAccess.getMbData    ().setTransformSize8x8  ( isTransformSize8x8  () );
  rcMbDataAccess.getMbData    ().setResidualPredFlag  ( getResidualPredFlag () );
}


Void IntMbTempData::loadChromaData( IntMbTempData& rcMbTempData )
{
  memcpy( get(CIdx(0)), rcMbTempData.get(CIdx(0)), sizeof(TCoeff)*128);
  setChromaPredMode( rcMbTempData.getChromaPredMode() );
  YuvMbBuffer::loadChroma( rcMbTempData );
  distU()  = rcMbTempData.distU();
  distV()  = rcMbTempData.distV();
  getTempYuvMbBuffer().loadChroma( rcMbTempData.getTempYuvMbBuffer() );
}


H264AVC_NAMESPACE_END
