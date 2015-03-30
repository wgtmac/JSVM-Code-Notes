
#if !defined(AFX_SEQUENCEPARAMETERSET_H__66281283_5BFB_429A_B722_6DDE7A11D086__INCLUDED_)
#define AFX_SEQUENCEPARAMETERSET_H__66281283_5BFB_429A_B722_6DDE7A11D086__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <cstdio>

#include "H264AVCCommonLib/HeaderSymbolReadIf.h"
#include "H264AVCCommonLib/HeaderSymbolWriteIf.h"
#include "H264AVCCommonLib/ScalingMatrix.h"

// JVT-V068 HRD {
#include "H264AVCCommonLib/Vui.h"
// JVT-V068 HRD }

H264AVC_NAMESPACE_BEGIN

class CodingParameter;
class ResizeParameters;

class H264AVCCOMMONLIB_API SequenceParameterSet
{
protected:
  typedef struct
  {
    Bool bValid;
    UInt uiMaxMbPerSec;
    UInt uiMaxFrameSize;
    UInt uiMaxDPBSizeX2;
    UInt uiMaxBitRate;
    UInt uiMaxCPBSize;
    UInt uiMaxVMvRange;
    UInt uiMinComprRatio;
    UInt uiMaxMvsPer2Mb;
  } LevelLimit;

  SequenceParameterSet          ();
  virtual ~SequenceParameterSet ();

public:
  static ErrVal create                    ( SequenceParameterSet*& rpcSPS );
  ErrVal        destroy                   ();

	SequenceParameterSet& operator = ( const SequenceParameterSet& rcSPS );

  Bool    doesFulfillMGSConstraint    ( const SequenceParameterSet& rcSPS )  const;
  ErrVal  copySPSDataForMGSEnhancement( const SequenceParameterSet& rcSPS );

  static UInt   getLevelIdc               ( UInt uiMbY, UInt uiMbX, UInt uiOutFreq, UInt uiMvRange, UInt uiNumRefPic, UInt uiRefLayerMbs );
  UInt          getMaxDPBSize             () const;

  static Short  getMaxIntMvVer            ( UInt uiLevelIdc, Bool bField ) { return m_aLevelLimit[uiLevelIdc].uiMaxVMvRange / (bField?2:1); }
  static Short  getMaxIntMvHor            () { return 8192; }

  Bool                  isSubSetSPS                           ()          const { return m_eNalUnitType == NAL_UNIT_SUBSET_SPS; }
  NalUnitType           getNalUnitType                        ()          const { return m_eNalUnitType; }
  UInt                  getDependencyId                       ()          const { return m_uiDependencyId; }
  Profile               getProfileIdc                         ()          const { return m_eProfileIdc;}
  Bool                  getConstrainedSet0Flag                ()          const { return m_bConstrainedSet0Flag; }
  Bool                  getConstrainedSet1Flag                ()          const { return m_bConstrainedSet1Flag; }
  Bool                  getConstrainedSet2Flag                ()          const { return m_bConstrainedSet2Flag; }
  Bool                  getConstrainedSet3Flag                ()          const { return m_bConstrainedSet3Flag; }
  UInt                  getLevelIdc                           ()          const { return m_uiLevelIdc;}
  UInt                  getConvertedLevelIdc                  ()          const
  {
    if( ( m_eProfileIdc == BASELINE_PROFILE || m_eProfileIdc == MAIN_PROFILE || m_eProfileIdc == EXTENDED_PROFILE ) && m_uiLevelIdc == 11 && m_bConstrainedSet3Flag )
    {
      return 9;
    }
    return m_uiLevelIdc;
  }
  UInt                  getSeqParameterSetId                  ()          const { return m_uiSeqParameterSetId;}
  Bool                  getSeqScalingMatrixPresentFlag        ()          const { return m_bSeqScalingMatrixPresentFlag; }
  const ScalingMatrix&  getSeqScalingMatrix                   ()          const { return m_cSeqScalingMatrix; }
  ScalingMatrix&        getSeqScalingMatrix                   ()                { return m_cSeqScalingMatrix; }
  UInt                  getLog2MaxFrameNum                    ()          const { return m_uiLog2MaxFrameNum;}
	UInt                  getPicOrderCntType                    ()          const { return m_uiPicOrderCntType;}
  UInt                  getLog2MaxPicOrderCntLsb              ()          const { return m_uiLog2MaxPicOrderCntLsb;}
  Bool                  getDeltaPicOrderAlwaysZeroFlag        ()          const { return m_bDeltaPicOrderAlwaysZeroFlag;}
  Int                   getOffsetForNonRefPic                 ()          const { return m_iOffsetForNonRefPic;}
  Int                   getOffsetForTopToBottomField          ()          const { return m_iOffsetForTopToBottomField;}
  UInt                  getNumRefFramesInPicOrderCntCycle     ()          const { return m_uiNumRefFramesInPicOrderCntCycle; }
  Int                   getOffsetForRefFrame                  ( UInt ui ) const { return m_piOffsetForRefFrame[ui]; }
  UInt                  getNumRefFrames                       ()          const { return m_uiNumRefFrames;}
  Bool                  getGapsInFrameNumValueAllowedFlag     ()          const { return m_bGapsInFrameNumValueAllowedFlag;}
  UInt                  getFrameWidthInMbs                    ()          const { return m_uiFrameWidthInMbs;}
  UInt                  getFrameHeightInMbs                   ()          const { return m_uiFrameHeightInMbs;}
  Bool                  getDirect8x8InferenceFlag             ()          const { return m_bDirect8x8InferenceFlag;}
  UInt                  getMbInFrame                          ()          const { return m_uiFrameWidthInMbs * m_uiFrameHeightInMbs;}
	Bool                  getFrameMbsOnlyFlag()                             const { return m_bFrameMbsOnlyFlag; }
	Bool                  getMbAdaptiveFrameFieldFlag()                     const { return m_bMbAdaptiveFrameFieldFlag; }

  Void  setNalUnitType                        ( NalUnitType e )           { m_eNalUnitType                          = e;  }
  Void  setDependencyId                       ( UInt        ui )          { m_uiDependencyId                        = ui; }
  Void  setProfileIdc                         ( Profile     e  )          { m_eProfileIdc                           = e;  }
  Void  setConstrainedSet0Flag                ( Bool        b  )          { m_bConstrainedSet0Flag                  = b;  }
  Void  setConstrainedSet1Flag                ( Bool        b  )          { m_bConstrainedSet1Flag                  = b;  }
  Void  setConstrainedSet2Flag                ( Bool        b  )          { m_bConstrainedSet2Flag                  = b;  }
  Void  setConstrainedSet3Flag                ( Bool        b  )          { m_bConstrainedSet3Flag                  = b;  }
  Void  setLevelIdc                           ( UInt        ui )          { m_uiLevelIdc                            = ui; }
  Void  setConvertedLevelIdc                  ( UInt        ui )
  {
    if( ( m_eProfileIdc == BASELINE_PROFILE || m_eProfileIdc == MAIN_PROFILE || m_eProfileIdc == EXTENDED_PROFILE ) && ui == 9 )
    {
      m_uiLevelIdc            = 11;
      m_bConstrainedSet3Flag  = true;
    }
    else
    {
      m_uiLevelIdc = ui;
    }
  }
  Void  setSeqParameterSetId                  ( UInt        ui )          { m_uiSeqParameterSetId                   = ui; }
  Void  setSeqScalingMatrixPresentFlag        ( Bool        b  )          { m_bSeqScalingMatrixPresentFlag          = b;  }
  Void  setLog2MaxFrameNum                    ( UInt        ui )          { m_uiLog2MaxFrameNum                     = ui; }
  Void  setPicOrderCntType                    ( UInt        ui )          { m_uiPicOrderCntType                     = ui; }
  Void  setLog2MaxPicOrderCntLsb              ( UInt        ui )          { m_uiLog2MaxPicOrderCntLsb               = ui; }
  Void  setDeltaPicOrderAlwaysZeroFlag        ( Bool        b  )          { m_bDeltaPicOrderAlwaysZeroFlag          = b;  }
  Void  setOffsetForNonRefPic                 ( Int         i  )          { m_iOffsetForNonRefPic                   = i;  }
  Void  setOffsetForTopToBottomField          ( Int         i  )          { m_iOffsetForTopToBottomField            = i;  }
  Void  setNumRefFramesInPicOrderCntCycle     ( UInt        ui )          { m_uiNumRefFramesInPicOrderCntCycle      = ui; }
  Void  setOffsetForRefFrame                  ( UInt        ui,
		                                            Int         i  )          { m_piOffsetForRefFrame[ui]               = i;  }
  Void  setNumRefFrames                       ( UInt        ui )          { m_uiNumRefFrames                        = ui; }
  Void  setGapsInFrameNumValueAllowedFlag     ( Bool        b  )          { m_bGapsInFrameNumValueAllowedFlag       = b;  }
  Void  setFrameWidthInMbs                    ( UInt        ui )          { m_uiFrameWidthInMbs                     = ui; }
  Void  setFrameHeightInMbs                   ( UInt        ui )          { m_uiFrameHeightInMbs                    = ui; }
  Void  setDirect8x8InferenceFlag             ( Bool        b  )          { m_bDirect8x8InferenceFlag               = b;  }

  Void setMGSVect                             ( UInt ui, UInt uiVect )    { m_uiMGSVect[ui] = uiVect; }
  UInt getMGSCoeffStart                       ( UInt uiNum )        const { return uiNum ? getMGSCoeffStart( uiNum - 1 ) + m_uiMGSVect[uiNum - 1] : 0; }
  UInt getMGSCoeffStop                        ( UInt uiNum )        const { return         getMGSCoeffStart( uiNum + 1 );                              }
  UInt getNumberOfQualityLevelsCGSSNR         () const
  {
    UInt uiQLs = 0;
    for( ; getMGSCoeffStop( uiQLs ) != 16; uiQLs++ ) {}
    return uiQLs + 1;
  }

  Void setInterlayerDeblockingPresent ( Bool b ) { m_bInterlayerDeblockingPresent = b ;}
  Bool getInterlayerDeblockingPresent () const    { return m_bInterlayerDeblockingPresent; }

  Void  setFrameMbsOnlyFlag                   ( Bool        b  )          { m_bFrameMbsOnlyFlag                     = b;  }
	Void  setMbAdaptiveFrameFieldFlag           ( Bool        b  )          { m_bMbAdaptiveFrameFieldFlag             = b;  }

  ErrVal initOffsetForRefFrame( UInt uiSize )
  {
    ROT ( uiSize<1 );

    RNOK( m_piOffsetForRefFrame.uninit() );
    RNOK( m_piOffsetForRefFrame.init( uiSize ) );

    return Err::m_nOK;
  }

  Void  setAllocFrameMbsX( UInt ui )  { m_uiAllocFrameMbsX = ui; }
  Void  setAllocFrameMbsY( UInt ui )  { m_uiAllocFrameMbsY = ui; }

  ErrVal write( HeaderSymbolWriteIf*  pcWriteIf )       const;
  ErrVal read ( HeaderSymbolReadIf*   pcReadIf,
                NalUnitType           eNalUnitType,
                Bool&                 rbCompletelyParsed );

// TMM_ESS {
  Void setResizeParameters    ( const ResizeParameters& rcResizeParameters );

  Void setExtendedSpatialScalability ( UInt ui ) { m_uiExtendedSpatialScalability = ui ;}
  UInt getExtendedSpatialScalability () const    { return m_uiExtendedSpatialScalability; }
  //JVT-W046 {
  Void  setChromaFormatIdc      ( UInt ui ) { m_uiChromaFormatIdc       = ui; }
  Void  setSeparateColourPlaneFlag( Bool b ) { m_bSeparateColourPlaneFlag = b; }
  Void  setBitDepthLumaMinus8   ( UInt ui ) { m_uiBitDepthLumaMinus8    = ui; }
  Void  setBitDepthChromaMinus8 ( UInt ui ) { m_uiBitDepthChromaMinus8  = ui; }
  Void  setTransformBypassFlag  ( Bool b  ) { m_bTransformBypassFlag    = b; }
  UInt  getChromaFormatIdc      ()  const { return m_uiChromaFormatIdc; }
  Bool  getSeparateColourPlaneFlag() const { return m_bSeparateColourPlaneFlag; }
  UInt  getBitDepthLumaMinus8   ()  const { return m_uiBitDepthLumaMinus8; }
  UInt  getBitDepthChromaMinus8 ()  const { return m_uiBitDepthChromaMinus8; }
  Bool  getTransformBypassFlag  ()  const { return m_bTransformBypassFlag; }
  Bool  getAVCHeaderRewriteFlag ()                const { return m_bAVCHeaderRewriteFlag; }
  Void  setAVCHeaderRewriteFlag ( Bool b )        { m_bAVCHeaderRewriteFlag = b; }
  Void  setBaseChromaPhaseXPlus1 ( UInt ui)       { m_uiBaseChromaPhaseXPlus1 = ui; }
  Void  setBaseChromaPhaseYPlus1 ( UInt ui)       { m_uiBaseChromaPhaseYPlus1 = ui; }
  UInt  getBaseChromaPhaseXPlus1 () const         { return m_uiBaseChromaPhaseXPlus1 ; }
  UInt  getBaseChromaPhaseYPlus1 () const         { return m_uiBaseChromaPhaseYPlus1 ; }
  Void  setChromaPhaseXPlus1 ( UInt ui)       { m_uiChromaPhaseXPlus1 = ui; }
  Void  setChromaPhaseYPlus1 ( UInt ui)       { m_uiChromaPhaseYPlus1 = ui; }
  Bool  getChromaPhaseXPlus1Flag () const         { return ( m_uiChromaPhaseXPlus1 > 0 ); }
  UInt  getChromaPhaseYPlus1 () const         { return m_uiChromaPhaseYPlus1 ; }
  Int   getChromaPhaseX           () const  { return (Int)m_uiChromaPhaseXPlus1 - 1; }
  Int   getChromaPhaseY           () const  { return (Int)m_uiChromaPhaseYPlus1 - 1; }
  Int   getScaledBaseLeftOffset   () const { return m_iScaledBaseLeftOffset; }
  Int   getScaledBaseTopOffset    () const { return m_iScaledBaseTopOffset; }
  Int   getScaledBaseRightOffset  () const { return m_iScaledBaseRightOffset; }
  Int   getScaledBaseBottomOffset () const { return m_iScaledBaseBottomOffset; }

  UInt  getAllocFrameMbsX         () const { return m_uiAllocFrameMbsX;  }
  UInt  getAllocFrameMbsY         () const { return m_uiAllocFrameMbsY; }

  //JVT-W046 }
  // JVT-V035
  Bool getTCoeffLevelPredictionFlag ()                       const { return m_bAVCRewriteFlag; }
  Bool getAVCAdaptiveRewriteFlag ()               const { return m_bAVCAdaptiveRewriteFlag; }
  Void setAVCRewriteFlag( Bool b )                { m_bAVCRewriteFlag = b; }
  Void setAVCAdaptiveRewriteFlag ( Bool b )
  {
	  if( getTCoeffLevelPredictionFlag() == false && b == true )
		  printf("WARNING: Setting AVCAdaptiveRewriteFlag when AVCRewriteFlag is false.\n");
	  m_bAVCAdaptiveRewriteFlag = b;
  }

  // TMM_ESS }

  // JVT-V068 HRD {
  VUI*  getVUI                     () const { return m_pcVUI; }
  Void  setVUI                     ( VUI* pcVUI )              { delete m_pcVUI; m_pcVUI = pcVUI; }
  Void  setVUI                     ( SequenceParameterSet* pcSPS );
  UInt          getMaxCPBSize() const;
  UInt          getMaxBitRate() const;
  // JVT-V068 HRD }

  UInt  getMaxSliceSize       ( Bool bFieldPic )  const;
  UInt  getMaxMVsPer2Mb       ()                  const;
  Bool  getBiPred8x8Disabled  ()                  const;

  Bool getSVCVUIParametersPresentFlag()            const { return m_bSVCVUIParametersPresentFlag;      }
	Bool getAdditionalExtension2Flag()               const { return m_bAdditionalExtension2Flag;         }
  Void setSVCVUIParametersPresentFlag ( Bool b )         {  m_bSVCVUIParametersPresentFlag  = b;       }
	Void setAdditionalExtension2Flag    ( Bool b )         {  m_bAdditionalExtension2Flag     = b;       }

  UInt  getFrameCropLeftOffset  ()  const { return m_uiFrameCropLeftOffset;   }
  UInt  getFrameCropRightOffset ()  const { return m_uiFrameCropRightOffset;  }
  UInt  getFrameCropTopOffset   ()  const { return m_uiFrameCropTopOffset;    }
  UInt  getFrameCropBottomOffset()  const { return m_uiFrameCropBottomOffset; }

  Void  setFrameCropLeftOffset  ( UInt ui ) { m_uiFrameCropLeftOffset   = ui; }
  Void  setFrameCropRightOffset ( UInt ui ) { m_uiFrameCropRightOffset  = ui; }
  Void  setFrameCropTopOffset   ( UInt ui ) { m_uiFrameCropTopOffset    = ui; }
  Void  setFrameCropBottomOffset( UInt ui ) { m_uiFrameCropBottomOffset = ui; }

protected:
	ErrVal xReadPicOrderCntInfo         ( HeaderSymbolReadIf* pcReadIf );
  static ErrVal xGetLevelLimit        ( const LevelLimit*&    rpcLevelLimit,
                                        Int                   iLevelIdc );
  ErrVal        xReadFrext            ( HeaderSymbolReadIf*   pcReadIf );
  ErrVal        xWriteFrext           ( HeaderSymbolWriteIf*  pcWriteIf ) const;


protected:
  NalUnitType   m_eNalUnitType;               //nal_unit
  UInt          m_uiDependencyId;                  //nal_unit_header_svc_extension
  Profile       m_eProfileIdc;                           //seq_parameter_set_rbsp
  Bool          m_bConstrainedSet0Flag;         //seq_parameter_set_rbsp
  Bool          m_bConstrainedSet1Flag;         //seq_parameter_set_rbsp
  Bool          m_bConstrainedSet2Flag;         //seq_parameter_set_rbsp
	Bool          m_bConstrainedSet3Flag;       //seq_parameter_set_rbsp
  UInt          m_uiLevelIdc;                              //seq_parameter_set_rbsp
  UInt          m_uiSeqParameterSetId;            //seq_parameter_set_rbsp
  UInt          m_uiChromaFormatIdc;//JVT-W046    //seq_parameter_set_rbsp
  Bool          m_bSeparateColourPlaneFlag;   //seq_parameter_set_rbsp
  UInt          m_uiBitDepthLumaMinus8;         //seq_parameter_set_rbsp
  UInt          m_uiBitDepthChromaMinus8;     //seq_parameter_set_rbsp
  Bool          m_bTransformBypassFlag;          //seq_parameter_set_rbsp      --->  qpprime_y_zero_transform_bypass_flag
	Bool          m_bSeqScalingMatrixPresentFlag;    //seq_parameter_set_rbsp
  ScalingMatrix m_cSeqScalingMatrix;          //保存scaling_list的类   我猜是SPS里的seq_scaling_list_present_flag[ i ]
  UInt          m_uiLog2MaxFrameNum;           //seq_parameter_set_rbsp
	UInt          m_uiPicOrderCntType;               //seq_parameter_set_rbsp
  UInt          m_uiLog2MaxPicOrderCntLsb;   //seq_parameter_set_rbsp
  Bool          m_bDeltaPicOrderAlwaysZeroFlag;    //seq_parameter_set_rbsp
  Int           m_iOffsetForNonRefPic;                //seq_parameter_set_rbsp
  Int           m_iOffsetForTopToBottomField;   //seq_parameter_set_rbsp
  UInt          m_uiNumRefFramesInPicOrderCntCycle;   //seq_parameter_set_rbsp
  DynBuf<Int>   m_piOffsetForRefFrame;      //SPS中保存offset_for_ref_frame[ i ]的类
  UInt          m_uiNumRefFrames;                    //seq_parameter_set_rbsp    --->  max_num_ref_frames
  Bool		   m_bGapsInFrameNumValueAllowedFlag;   //seq_parameter_set_rbsp 
  UInt          m_uiFrameWidthInMbs;              //seq_parameter_set_rbsp 
  UInt          m_uiFrameHeightInMbs;             //seq_parameter_set_rbsp 
  Bool          m_bDirect8x8InferenceFlag;       //seq_parameter_set_rbsp 

// TMM_ESS {
  UInt          m_uiExtendedSpatialScalability;              //seq_parameter_set_svc_extension
  UInt          m_uiChromaPhaseXPlus1;                        //seq_parameter_set_svc_extension
  UInt          m_uiChromaPhaseYPlus1;                        //seq_parameter_set_svc_extension
  UInt          m_uiBaseChromaPhaseXPlus1;                //seq_parameter_set_svc_extension
  UInt          m_uiBaseChromaPhaseYPlus1;                //seq_parameter_set_svc_extension
  Int           m_iScaledBaseLeftOffset;                           //seq_parameter_set_svc_extension
  Int           m_iScaledBaseTopOffset;                           //seq_parameter_set_svc_extension
  Int           m_iScaledBaseRightOffset;                        //seq_parameter_set_svc_extension
  Int           m_iScaledBaseBottomOffset;                    //seq_parameter_set_svc_extension

// TMM_ESS }

  Bool          m_bInterlayerDeblockingPresent;            //seq_parameter_set_svc_extension

// VW {
	UInt					m_auiNumRefIdxUpdateActiveDefault[2];
// VW }

  UInt          m_uiMGSVect[16];
	Bool          m_bFrameMbsOnlyFlag;                          //seq_parameter_set_rbsp 
	Bool          m_bMbAdaptiveFrameFieldFlag;            //seq_parameter_set_rbsp 

  Bool          m_bAVCRewriteFlag;          // V-035
  Bool          m_bAVCAdaptiveRewriteFlag;
  Bool          m_bAVCHeaderRewriteFlag;    // JVT-W046

  // JVT-V068 HRD {
  VUI*          m_pcVUI;
  // JVT-V068 HRD }

	Bool m_bSVCVUIParametersPresentFlag;       //seq_parameter_set_rbsp 
	Bool m_bAdditionalExtension2Flag;

	/*
	比起标准中的seq_parameter_set_rbsp( )  缺少 
	frame_cropping_flag 
	constraint_set4_flag
	constraint_set5_flag

	比起标准中的seq_parameter_set_svc_extension( )  缺少
	seq_tcoeff_level_prediction_flag
	adaptive_tcoeff_level_prediction_flag
	slice_header_restriction_flag
	*/
  UInt          m_uiFrameCropLeftOffset;               //seq_parameter_set_rbsp 
  UInt          m_uiFrameCropRightOffset;            //seq_parameter_set_rbsp 
  UInt          m_uiFrameCropTopOffset;               //seq_parameter_set_rbsp 
  UInt          m_uiFrameCropBottomOffset;         //seq_parameter_set_rbsp 

  UInt          m_uiAllocFrameMbsX;
  UInt          m_uiAllocFrameMbsY;

private:
  static const LevelLimit m_aLevelLimit[52];
};


H264AVC_NAMESPACE_END


#endif // !defined(AFX_SEQUENCEPARAMETERSET_H__66281283_5BFB_429A_B722_6DDE7A11D086__INCLUDED_)
