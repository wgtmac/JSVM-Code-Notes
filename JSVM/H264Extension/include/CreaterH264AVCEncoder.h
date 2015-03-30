
#if !defined(AFX_CREATERH264AVCENCODER_H__0366BFA9_45D9_4834_B404_8DE3914C1E58__INCLUDED_)
#define AFX_CREATERH264AVCENCODER_H__0366BFA9_45D9_4834_B404_8DE3914C1E58__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000



H264AVC_NAMESPACE_BEGIN



class H264AVCEncoder;
class MbData;
class BitWriteBuffer;
class Transform;
class YuvBufferCtrl;
class QuarterPelFilter;
class ParameterSetMng;
class LoopFilter;
class SampleWeighting;
class PocCalculator;

class BitCounter;
class SliceEncoder;
class UvlcWriter;
class MbCoder;
class MbEncoder;
class IntraPredictionSearch;
class CodingParameter;
class CabacWriter;
class NalUnitEncoder;
class Distortion;
class MotionEstimation;
class MotionEstimationQuarterPel;
class RateDistortion;
class RateDistortionRateConstraint;
class History;
class LayerEncoder;
class XDistortion;
class ControlMngH264AVCEncoder;
class ReconstructionBypass;
class PicEncoder;
// JVT-V068 {
class Scheduler;
class SequenceParameterSet;
// JVT-V068 }


class H264AVCENCODERLIB_API CreaterH264AVCEncoder
{
protected:
	CreaterH264AVCEncoder();
	virtual ~CreaterH264AVCEncoder();

public:
  static ErrVal create  ( CreaterH264AVCEncoder*& rpcCreaterH264AVCEncoder );
  ErrVal        destroy ();

  ErrVal init               ( CodingParameter*    pcCodingParameter);
  ErrVal uninit             ();
  ErrVal writeParameterSets ( ExtBinDataAccessor* pcExtBinDataAccessor,
                              SequenceParameterSet*& rpcAVCSPS,
                              Bool&               rbMoreSets );

// JVT-V068 {
  ErrVal writeAVCCompatibleHRDSEI( ExtBinDataAccessor* pcExtBinDataAccessor, SequenceParameterSet*pcAVCSPS );
// JVT-V068 }

  ErrVal process(  ExtBinDataAccessorList&  rcExtBinDataAccessorList,
                   PicBuffer*               apcOriginalPicBuffer    [MAX_LAYERS],
                   PicBuffer*               apcReconstructPicBuffer [MAX_LAYERS],
                   PicBufferList*           apcPicBufferOutputList,
                   PicBufferList*           apcPicBufferUnusedList );

  ErrVal finish (  ExtBinDataAccessorList&  rcExtBinDataAccessorList,
                   PicBufferList*           apcPicBufferOutputList,
                   PicBufferList*           apcPicBufferUnusedList,
                   UInt&                    ruiNumCodedFrames,
                   Double&                  rdHighestLayerOutputRate );
  Bool getScalableSeiMessage ( void );
	Void SetVeryFirstCall ( void );
  // JVT-S080 LMI {
  ErrVal xWriteScalableSEILayersNotPresent( ExtBinDataAccessor* pcExtBinDataAccessor, UInt uiInputLayers, UInt* m_layer_id);
  ErrVal xWriteScalableSEIDependencyChange( ExtBinDataAccessor* pcExtBinDataAccessor, UInt uiNumLayers, UInt* uiLayerId, Bool* pbLayerDependencyInfoPresentFlag,
												  UInt* uiNumDirectDependentLayers, UInt** puiDirectDependentLayerIdDeltaMinus1, UInt* puiLayerDependencyInfoSrcLayerIdDeltaMinus1);
  // JVT-S080 LMI }
  // JVT-W043
  CodingParameter*  getCodingParameter( void );

protected:
  ErrVal xCreateEncoder();

protected:
  H264AVCEncoder*           m_pcH264AVCEncoder;           //SVC编码    AVCMode() = 0

  LayerEncoder*             m_apcLayerEncoder         [MAX_LAYERS];
  SliceEncoder*             m_pcSliceEncoder;
  ControlMngH264AVCEncoder* m_pcControlMng;
  BitWriteBuffer*           m_pcBitWriteBuffer;
  BitCounter*               m_pcBitCounter;
  NalUnitEncoder*           m_pcNalUnitEncoder;

  UvlcWriter*               m_pcUvlcWriter;              //写比特流实际用的变量
  UvlcWriter*               m_pcUvlcTester;
  MbCoder*                  m_pcMbCoder;                //宏块编码
  LoopFilter*               m_pcLoopFilter;                //环路去块滤波
  MbEncoder*                m_pcMbEncoder;          //宏块编码 派生类
  Transform*                m_pcTransform;                //变换
  IntraPredictionSearch*    m_pcIntraPrediction;      //帧内预测
  YuvBufferCtrl*            m_apcYuvFullPelBufferCtrl [MAX_LAYERS];
  YuvBufferCtrl*            m_apcYuvHalfPelBufferCtrl [MAX_LAYERS];
  QuarterPelFilter*         m_pcQuarterPelFilter;                //1/4像素滤波
  CodingParameter*          m_pcCodingParameter;         //编码参数
  ParameterSetMng*          m_pcParameterSetMng;       //参数集管理
  PocCalculator*            m_apcPocCalculator        [MAX_LAYERS];      //计算POC
  SampleWeighting*          m_pcSampleWeighting;      //采样？？？
  CabacWriter*              m_pcCabacWriter;                     //CABAC熵编码
  XDistortion*              m_pcXDistortion;
  MotionEstimation*         m_pcMotionEstimation;      //运动估计
  RateDistortion*           m_pcRateDistortion;                 // 率失真
  History*                  m_pcHistory;
  ReconstructionBypass*     m_pcReconstructionBypass;        //旁路重建
  PicEncoder*               m_pcPicEncoder;                        //普通AVC编码      AVCMode() = 0
  Bool                      m_bTraceEnable;
  // JVT-V068 HRD {
  StatBuf<Scheduler*, MAX_SCALABLE_LAYERS> m_apcScheduler;
  // JVT-V068 HRD }
};

H264AVC_NAMESPACE_END


#endif // !defined(AFX_CREATERH264AVCENCODER_H__0366BFA9_45D9_4834_B404_8DE3914C1E58__INCLUDED_)
