
#include "H264AVCCommonLib.h"

#include "H264AVCCommonLib/Tables.h"
#include "H264AVCCommonLib/Transform.h"

H264AVC_NAMESPACE_BEGIN

// for SVC to AVC rewrite
CoeffLevelPred::CoeffLevelPred()
{
}

CoeffLevelPred::~CoeffLevelPred()
{
}

ErrVal
CoeffLevelPred::ScaleCoeffLevels( TCoeff* piCoeff, TCoeff* piRefCoeff, UInt uiQp, UInt uiRefQp, UInt uiNumCoeffs )
{
  // DETERMINE THE SCALING FACTOR
  const Int aiScaleFactor[6]  = { 8, 9, 10, 11, 13, 14 };
  Int       iDeltaQp          = uiRefQp - uiQp + 54;
  Int       iScale            = aiScaleFactor[ iDeltaQp % 6 ];
  Int       iShift            = iDeltaQp / 6;
  // PREDICT THE COEFFICIENTS
  for( UInt n = 0; n < uiNumCoeffs; n++ )
  {
    piCoeff[n] = ( ( iScale * piRefCoeff[n] ) << iShift ) >> 12;
  }
  return Err::m_nOK;
}

Transform::Transform()
: m_bClip( true )
, m_storeCoeffFlag (true)
{
}

Transform::~Transform()
{
}


ErrVal Transform::create( Transform*& rpcTransform )
{
  rpcTransform = new Transform;

  ROT( NULL == rpcTransform );

  return Err::m_nOK;
}


ErrVal Transform::destroy()
{
  delete this;

  return Err::m_nOK;
}

ErrVal Transform::invTransform4x4Blk( Pel* puc, Int iStride, TCoeff* piCoeff )
{
  xInvTransform4x4Blk( puc, iStride, piCoeff );

  return Err::m_nOK;
}



ErrVal Transform::invTransformDcCoeff( TCoeff* piCoeff, Int iQpScale )
{
  Int aai[4][4];
  Int tmp1, tmp2;
  Int x, y;

  TCoeff *piWCoeff = piCoeff;

  for( x = 0; x < 4; x++, piCoeff += 0x10 )
  {
    tmp1 = piCoeff[0x00] + piCoeff[0x80];
    tmp2 = piCoeff[0xc0] + piCoeff[0x40];

    aai[x][0] = tmp1 + tmp2;
	  aai[x][3] = tmp1 - tmp2;

    tmp1 = piCoeff[0x00] - piCoeff[0x80];
    tmp2 = piCoeff[0x40] - piCoeff[0xc0];

    aai[x][1] = tmp1 + tmp2;
    aai[x][2] = tmp1 - tmp2;
  }

  for( y = 0; y < 4; y++ )
  {
    tmp1 = aai[0][y] + aai[2][y];
    tmp2 = aai[3][y] + aai[1][y];

    piWCoeff[0x00] = (( tmp1 + tmp2 ) * iQpScale + 2 ) >> 2;
	  piWCoeff[0x30] = (( tmp1 - tmp2 ) * iQpScale + 2 ) >> 2;

    tmp1 = aai[0][y] - aai[2][y];
    tmp2 = aai[1][y] - aai[3][y];

    piWCoeff[0x10] = (( tmp1 + tmp2 ) * iQpScale + 2 ) >> 2;
    piWCoeff[0x20] = (( tmp1 - tmp2 ) * iQpScale + 2 ) >> 2;

    piWCoeff += 0x40;
  }

  return Err::m_nOK;
}

ErrVal Transform::invTransformDcCoeff( TCoeff* piCoeff, const Int iQpScale, const Int iQpPer )
{
  Int aai[4][4];
  Int tmp1, tmp2;
  Int x, y;

  TCoeff *piWCoeff = piCoeff;

  for( x = 0; x < 4; x++, piCoeff += 0x10 )
  {
    tmp1 = piCoeff[0x00] + piCoeff[0x80];
    tmp2 = piCoeff[0xc0] + piCoeff[0x40];

    aai[x][0] = tmp1 + tmp2;
    aai[x][3] = tmp1 - tmp2;

    tmp1 = piCoeff[0x00] - piCoeff[0x80];
    tmp2 = piCoeff[0x40] - piCoeff[0xc0];

    aai[x][1] = tmp1 + tmp2;
    aai[x][2] = tmp1 - tmp2;
  }

  if( iQpPer < 6)
  {
    const Int iAdd      = 1 << (5-iQpPer);
    const Int iShift    = 6-iQpPer;

    for( y = 0; y < 4; y++ )
    {
      tmp1 = aai[0][y] + aai[2][y];
      tmp2 = aai[3][y] + aai[1][y];

      piWCoeff[0x00] = ((( tmp1 + tmp2 ) * iQpScale + iAdd ) >> iShift);
      piWCoeff[0x30] = ((( tmp1 - tmp2 ) * iQpScale + iAdd ) >> iShift);

      tmp1 = aai[0][y] - aai[2][y];
      tmp2 = aai[1][y] - aai[3][y];

      piWCoeff[0x10] = ((( tmp1 + tmp2 ) * iQpScale + iAdd ) >> iShift );
      piWCoeff[0x20] = ((( tmp1 - tmp2 ) * iQpScale + iAdd ) >> iShift );

      piWCoeff += 0x40;
    }
  }
  else
  {
    const Int iShift    = iQpPer-6;

    for( y = 0; y < 4; y++ )
    {
      tmp1 = aai[0][y] + aai[2][y];
      tmp2 = aai[3][y] + aai[1][y];

      piWCoeff[0x00] = (( tmp1 + tmp2 ) * iQpScale ) << iShift;
      piWCoeff[0x30] = (( tmp1 - tmp2 ) * iQpScale ) << iShift;

      tmp1 = aai[0][y] - aai[2][y];
      tmp2 = aai[1][y] - aai[3][y];

      piWCoeff[0x10] = (( tmp1 + tmp2 ) * iQpScale ) << iShift;
      piWCoeff[0x20] = (( tmp1 - tmp2 ) * iQpScale ) << iShift;

      piWCoeff += 0x40;
    }
  }

  return Err::m_nOK;
}

Void Transform::xForTransformLumaDc( TCoeff* piCoeff )		// 哈达玛变换
{
  Int aai[4][4];
  Int tmp1, tmp2;
  Int x, y;

  TCoeff *piWCoeff = piCoeff;

  for( x = 0; x < 4; x++, piCoeff += 0x10 )		// 每16个系数取出一个  ==> DC    piCoeff每次移动一列  因此是 += 0x10
  {
    tmp1 = piCoeff[0x00] + piCoeff[0x80];
    tmp2 = piCoeff[0xc0] + piCoeff[0x40];

    aai[x][0] = tmp1 + tmp2;
	  aai[x][3] = tmp1 - tmp2;

    tmp1 = piCoeff[0x00] - piCoeff[0x80];
    tmp2 = piCoeff[0x40] - piCoeff[0xc0];

    aai[x][1] = tmp1 + tmp2;
    aai[x][2] = tmp1 - tmp2;
  }

  for( y = 0; y < 4; y++ )
  {
    tmp1 = aai[0][y] + aai[2][y];
    tmp2 = aai[3][y] + aai[1][y];

    piWCoeff[0x00] = (tmp1 + tmp2)/2;
	  piWCoeff[0x30] = (tmp1 - tmp2)/2;

    tmp1 = aai[0][y] - aai[2][y];
    tmp2 = aai[1][y] - aai[3][y];

    piWCoeff[0x10] = (tmp1 + tmp2)/2;
    piWCoeff[0x20] = (tmp1 - tmp2)/2;

    piWCoeff += 0x40;
  }
}



Void Transform::xInvTransform4x4Blk( Pel* puc, Int iStride, TCoeff* piCoeff )
{

  Int aai[4][4];
  Int tmp1, tmp2;
  Int x, y;
  Int iStride2=2*iStride;
  Int iStride3=3*iStride;

  for( x = 0; x < 4; x++, piCoeff+=4 )
  {
    tmp1 = piCoeff[0] + piCoeff[2];
    tmp2 = (piCoeff[3]>>1) + piCoeff[1];

    aai[0][x] = tmp1 + tmp2;
	  aai[3][x] = tmp1 - tmp2;

    tmp1 = piCoeff[0] - piCoeff[2];
    tmp2 = (piCoeff[1]>>1) - piCoeff[3];

    aai[1][x] = tmp1 + tmp2;
    aai[2][x] = tmp1 - tmp2;
  }

  for( y = 0; y < 4; y++, puc ++ )
  {
    tmp1 =  aai[y][0] + aai[y][2];
    tmp2 = (aai[y][3]>>1) + aai[y][1];

    puc[0]        = gClip( xRound( tmp1 + tmp2) + puc[0]        );
	  puc[iStride3] = gClip( xRound( tmp1 - tmp2) + puc[iStride3] );

    tmp1 =  aai[y][0] - aai[y][2];
    tmp2 = (aai[y][1]>>1) - aai[y][3];

    puc[iStride]  = gClip( xRound( tmp1 + tmp2) + puc[iStride]  );
	  puc[iStride2] = gClip( xRound( tmp1 - tmp2) + puc[iStride2] );

  }
}



Void Transform::xInvTransform4x4BlkNoAc( Pel* puc, Int iStride, TCoeff* piCoeff )
{
  Int iDc = xRound( piCoeff[0] );

  ROFVS( iDc );

  for( Int y = 0; y < 4; y++ )
  {
    puc[0] = gClip( iDc + puc[0] );
    puc[1] = gClip( iDc + puc[0] );
    puc[2] = gClip( iDc + puc[0] );
    puc[3] = gClip( iDc + puc[0] );
    puc += iStride;
  }
}



Void Transform::xInvTransform4x4BlkNoAc( XPel* puc, Int iStride, TCoeff* piCoeff )
{
  Int iDc = xRound( piCoeff[0] );

  ROFVS( iDc );

  for( Int y = 0; y < 4; y++ )
  {
    puc[0] = xClip( iDc + puc[0] );
    puc[1] = xClip( iDc + puc[0] );
    puc[2] = xClip( iDc + puc[0] );
    puc[3] = xClip( iDc + puc[0] );
    puc += iStride;
  }
}


Void Transform::invTransformChromaDc( TCoeff* piCoeff )
{
  Int   tmp1, tmp2;
  Int   d00, d01, d10, d11;

  d00 = piCoeff[0];
  d10 = piCoeff[32];
  d01 = piCoeff[16];
  d11 = piCoeff[48];

  tmp1 = d00 + d11;
  tmp2 = d10 + d01;

  piCoeff[ 0] = ( tmp1 + tmp2 ) >> 5;
  piCoeff[48] = ( tmp1 - tmp2 ) >> 5;

  tmp1 = d00 - d11;
  tmp2 = d01 - d10;

  piCoeff[32] = ( tmp1 + tmp2 ) >> 5;
  piCoeff[16] = ( tmp1 - tmp2 ) >> 5;
}

Void Transform::xForTransformChromaDc( TCoeff* piCoeff )
{
  Int   tmp1, tmp2;
  Int   d00, d01, d10, d11;

  d00 = piCoeff[0];
  d10 = piCoeff[16];
  d01 = piCoeff[32];
  d11 = piCoeff[48];

  tmp1 = d00 + d11;
  tmp2 = d10 + d01;

  piCoeff[ 0] = (tmp1 + tmp2);
  piCoeff[48] = (tmp1 - tmp2);

  tmp1 = d00 - d11;
  tmp2 = d01 - d10;

  piCoeff[16] = (tmp1 + tmp2);
  piCoeff[32] = (tmp1 - tmp2);
}




Void Transform::xQuantDequantNonUniformLuma( TCoeff* piQCoeff, TCoeff* piCoeff, const QpParameter& rcQp, const UChar* pucScale, UInt& ruiDcAbs, UInt& ruiAcAbs )
{
  Int   iLevel = piCoeff[0];
  UInt  uiSign = ((UInt)iLevel)>>31;             //获得符号   0正 1负
  Int   iAdd   = ( 1 << 3 ) >> rcQp.per();
//  量化DC系数**************************************
  iLevel       = abs( iLevel ) * g_aaiQuantCoef[ rcQp.rem() ][0];
  if( pucScale )
  {
    iLevel     = ( iLevel << 4 ) / pucScale[0];
  }
  iLevel       = ( iLevel + 2 * rcQp.add() ) >> ( rcQp.bits() + 1 );   //[0][0]处量化值

  ruiDcAbs    += iLevel;
  iLevel       = ( uiSign ? -iLevel : iLevel );
  piQCoeff[0]  = iLevel;

//  量化AC系数 **********************************
  UInt uiAcAbs = 0;            //临时
  for( Int n = 1; n < 16; n++ )
  {
    iLevel      = piCoeff[n];
    Int iSign   = iLevel;

    iLevel      = abs( iLevel ) * g_aaiQuantCoef[rcQp.rem()][n];
    if( pucScale )               //  我觉得 scalinglist就是 先×16 再量化
    {
      iLevel    = ( iLevel << 4 ) / pucScale[n];
    }
    iLevel      = ( iLevel + rcQp.add() ) >> rcQp.bits();

    if( 0 != iLevel )             //反量化步骤
    {
      iSign       >>= 31;
      Int iDeScale  = g_aaiDequantCoef[rcQp.rem()][n];
      uiAcAbs      += iLevel;
      iLevel       ^= iSign;
      iLevel       -= iSign;
      piQCoeff[n]   = iLevel;
      if( pucScale )
      {
        piCoeff[n] = ( ( iLevel*iDeScale*pucScale[n] + iAdd ) << rcQp.per() ) >> 4;
      }
      else
      {
        piCoeff[n]  = iLevel*iDeScale << rcQp.per();
      }
    }
    else
    {
      piQCoeff[n] = 0;
      piCoeff [n] = 0;
    }
  }

  ruiAcAbs += uiAcAbs;      //与参数进行相加
  return;
}


Void Transform::xQuantDequantNonUniformChroma( TCoeff* piQCoeff,
                                               TCoeff* piCoeff,
                                               const QpParameter& rcQp,
                                               const UChar* pucScale,
                                               UInt& ruiDcAbs,
                                               UInt& ruiAcAbs )
{
  Int   iAdd    = ( 1 << 3 ) >> rcQp.per();
  {
    Int   iLevel  = piCoeff[0];
    UInt  uiSign  = ((UInt)iLevel)>>31;

    iLevel        = ( abs( iLevel ) * g_aaiQuantCoef[ rcQp.rem() ][0] );
    if( pucScale )
    {
      iLevel      = ( iLevel << 4 ) / pucScale[0];
    }
    iLevel        = ( iLevel + 2*rcQp.add() ) >> ( rcQp.bits() + 1 );

    ruiDcAbs   += iLevel;
    iLevel      = ( uiSign ? -iLevel : iLevel );
    piQCoeff[0] = iLevel;
    // dequantize DC also
    piCoeff [0] = iLevel * ( pucScale ? pucScale[0] : 16 ) * g_aaiDequantCoef[rcQp.rem()][0] << rcQp.per();
  }

  UInt uiAcAbs = 0;
  for( int n = 1; n < 16; n++ )
  {
    Int iLevel  = piCoeff[n];
    Int iSign   = iLevel;

    iLevel      = ( abs( iLevel ) * g_aaiQuantCoef[rcQp.rem()][n] );
    if( pucScale )
    {
      iLevel    = ( iLevel << 4 ) / pucScale[n];
    }
    iLevel = ( iLevel + rcQp.add() ) >> rcQp.bits();

    if( 0 != iLevel )
    {
      iSign      >>= 31;
      Int iDeScale = g_aaiDequantCoef[rcQp.rem()][n];
      uiAcAbs     += iLevel;
      iLevel      ^= iSign;
      iLevel      -= iSign;
      piQCoeff[n]  = iLevel;
      if( pucScale )
      {
        piCoeff[n] = ( ( iLevel*iDeScale*pucScale[n] + iAdd ) << rcQp.per() ) >> 4;
      }
      else
      {
        piCoeff[n]  = iLevel*iDeScale << rcQp.per();
      }
    }
    else
    {
      piQCoeff[n] = 0;
      piCoeff[n] = 0;
    }
  }

  ruiAcAbs += uiAcAbs;
  return;
}





Void Transform::xQuantDequantUniform4x4( TCoeff*                      piQCoeff,      // 传出：量化后
                                         TCoeff*                      piCoeff,                                                // 传入：变换后   传出：反量化后
                                         const QpParameter&           rcQp,
                                         const UChar*                 pucScale,
                                         UInt&                        ruiAbsSum )		// 传出： 系数绝对值和
{
  Int n     = 0;
  ruiAbsSum = 0;
  Int iAdd  = ( 1 << 3 ) >> rcQp.per();

  for( ; n < 16; n++ )
  {
    Int iLevel  = piCoeff[n];	// 变换后系数
    Int iSign   = iLevel;

    iLevel      = abs( iLevel ) * g_aaiQuantCoef[rcQp.rem()][n];		// |W|*MF
    if( pucScale )
    {
      iLevel    = ( iLevel << 4 ) / pucScale[n];
    }
    iLevel      = ( iLevel + rcQp.add() ) >> rcQp.bits();
	// 量化到此结束 结果在 iLevel 中
    if( 0 != iLevel )
    {
      iSign      >>= 31;		// int 是带符号的位移
      Int iDeScale = g_aaiDequantCoef[ rcQp.rem() ][ n ];		// Z*V
      ruiAbsSum   += iLevel;
      iLevel      ^= iSign;
      iLevel      -= iSign;
      piQCoeff[n]  = iLevel;		// 这几步是还原符号 ---- 保存量化后系数

      if( pucScale )
      {
        piCoeff[n]   = ( ( iLevel*iDeScale*pucScale[n] + iAdd ) << rcQp.per() ) >> 4;
      }
      else
      {
        piCoeff[n]   = iLevel*iDeScale << rcQp.per();
      }
    }
    else
    {
      piQCoeff[n] = 0;
      piCoeff [n] = 0;
    }
  }
}


// for SVC to AVC rewrite
ErrVal Transform::predict4x4Blk( TCoeff* piCoeff, TCoeff* piRefCoeff, UInt uiRefQp, UInt& ruiAbsSum  )
{
	// DECLARATIONS
	TCoeff cPredCoeff[16];

	// PREDICT THE COEFFICIENTS     根据  piRefCoeff 直接在变换域 预测出 cPredCoeff
	ScaleCoeffLevels( cPredCoeff, piRefCoeff, getLumaQp().value(), uiRefQp, 16 );                  // 量化后的系数

	// ADJUST THE TRANSMITTED COEFFICIENTS
	for( Int n=0; n<16; n++ )
	{
    ruiAbsSum -= (piCoeff[n] >0) ? piCoeff[n].getCoeff() : (-1*piCoeff[n].getCoeff());
		piCoeff[n] -= cPredCoeff[n];                  // 【预测的系数】 =  【当前块量化后的系数】 - 【baselayer块量化后的系数】
		ruiAbsSum += (piCoeff[n]>0) ? piCoeff[n].getCoeff() : (-1*piCoeff[n].getCoeff());
	}

	return Err::m_nOK;
}

ErrVal Transform::predict8x8Blk( TCoeff* piCoeff, TCoeff* piRefCoeff, UInt uiRefQp, UInt& ruiAbsSum  )
{
	// DECLARATIONS
	TCoeff cPredCoeff[64];

	// PREDICT THE COEFFICIENTS
    ScaleCoeffLevels( cPredCoeff, piRefCoeff, getLumaQp().value(), uiRefQp, 64 );


	// ADJUST THE TRANSMITTED COEFFICIENTS
	for( Int n=0; n<64; n++ )
	{
    ruiAbsSum -= (piCoeff[n] >0) ? piCoeff[n].getCoeff() : -1*piCoeff[n].getCoeff();
		piCoeff[n] -= cPredCoeff[n];
		ruiAbsSum += (piCoeff[n]>0) ? piCoeff[n].getCoeff() : -1*piCoeff[n].getCoeff();
	}

	return Err::m_nOK;
}

ErrVal Transform::predictMb16x16( TCoeff* piCoeff, TCoeff* piRef, UInt uiRefQp, UInt& ruiDcAbs, UInt& ruiAcAbs )
{
	UInt uiAbs = 0;

	for( UInt n=0; n<16; n++ )
		predict4x4Blk( &piCoeff[n<<4], &piRef[n<<4], uiRefQp, uiAbs );

	return Err::m_nOK;

}

ErrVal Transform::predictChromaBlocks( UInt uiComp, TCoeff* piCoeff, TCoeff* piRef, UInt uiRefQp, UInt& ruiDcAbs, UInt& ruiAcAbs )
{
	// DECLARATIONS
	TCoeff cScaledRef[64];
  int i;

	for( UInt x=0; x<0x40; x+=0x10 )
	{
    ScaleCoeffLevels( &cScaledRef[x], &piRef[x], getChromaQp( uiComp ).value(), uiRefQp, 16 );

		for( UInt n=0; n<16; n++ )
		{
			piCoeff[x+n] -= cScaledRef[x+n];
		}
  }

	// RECOMPUTE THE COEFFICIENT COUNTS
	ruiAcAbs = 0;
	ruiDcAbs = 0;

	for(i=0; i<64; i++ )
		ruiAcAbs += abs( (Int)piCoeff[i] );

	for( i=0; i<64; i+=16 )
		ruiDcAbs += abs( (Int)piCoeff[i] );

	ruiAcAbs -= ruiDcAbs;

	return Err::m_nOK;

}
ErrVal
Transform::predictScaledACCoeffs( UInt uiComp, TCoeff *piCoeff, TCoeff *piRef, UInt uiRefQp, const UChar* pucScale )
{

	// DECLARATIONS
	TCoeff cPredCoeff[64] = {0};
	UInt uiDcAbs=0, uiAcAbs=0;

	// Predict the chroma coefficients
  predictChromaBlocks( uiComp, cPredCoeff, piRef, uiRefQp, uiDcAbs, uiAcAbs  );

	for( UInt i=0; i<64; i++ )
		cPredCoeff[i] = -cPredCoeff[i];

	// Scale the coefficients
  const QpParameter& rcCQp = ( uiComp ? m_cCrQp : m_cCbQp );
  x4x4DequantChroma( &cPredCoeff[0x00], &cPredCoeff[0x00], rcCQp, pucScale );
  x4x4DequantChroma( &cPredCoeff[0x10], &cPredCoeff[0x10], rcCQp, pucScale );
  x4x4DequantChroma( &cPredCoeff[0x20], &cPredCoeff[0x20], rcCQp, pucScale );
  x4x4DequantChroma( &cPredCoeff[0x30], &cPredCoeff[0x30], rcCQp, pucScale );

	// Substitute
	for( UInt x=0x00; x<0x40; x+=0x10 )
		for( UInt n=1; n<16; n++ )
			piCoeff[x+n] = cPredCoeff[x+n];


	return Err::m_nOK;
}

ErrVal
Transform::predictScaledChromaCoeffs( TCoeff *piCoeff, TCoeff *piRef, UInt uiRefCbQp, UInt uiRefCrQp, const UChar* pucScaleCb, const UChar* pucScaleCr )
{
  // DECLARATIONS
  TCoeff  cPredCoeff[0x80] = {0};
  UInt    uiDcAbs=0, uiAcAbs=0;

  // Predict the chroma coefficients
  predictChromaBlocks( 0, cPredCoeff+0x00, piRef+0x00, uiRefCbQp, uiDcAbs, uiAcAbs  );
  predictChromaBlocks( 1, cPredCoeff+0x40, piRef+0x40, uiRefCrQp, uiDcAbs, uiAcAbs  );

  for( UInt uiBlkOff = 0x00; uiBlkOff < 0x80; uiBlkOff += 0x10 )
  {
    TCoeff* piPredBlk = cPredCoeff  + uiBlkOff;
    TCoeff* piCoefBlk = piCoeff     + uiBlkOff;

    for( UInt i0 = 0; i0 < 16; i0++ )
    {
      piPredBlk[i0] = -piPredBlk[i0];
    }
    const UChar*        pucScale  = ( uiBlkOff >= 0x40 ? pucScaleCr : pucScaleCb );
    const QpParameter&  rcQp      = ( uiBlkOff >= 0x40 ? m_cCrQp    : m_cCbQp    );
    x4x4DequantChroma( piPredBlk, piPredBlk, rcQp, pucScale );
    for( UInt i1 = 0; i1 < 16; i1++ )
    {
      piCoefBlk[i1] = piPredBlk[i1];
    }
  }

  return Err::m_nOK;
}


ErrVal Transform::addPrediction4x4Blk( TCoeff* piCoeff, TCoeff* piRefCoeff, UInt uiQp, UInt uiRefQp, UInt &uiCoded  )
{
	// DECLARATIONS
	TCoeff cPredCoeff[16];

	// PREDICT THE COEFFICIENTS
	ScaleCoeffLevels( cPredCoeff, piRefCoeff, uiQp, uiRefQp, 16 );

	// ADJUST THE TRANSMITTED COEFFICIENTS
	for( Int n=0; n<16; n++ )
	{
		piCoeff[n] += cPredCoeff[n];
		if( piCoeff[n] )
			uiCoded++;
	}

	return Err::m_nOK;
}

ErrVal Transform::addPrediction8x8Blk( TCoeff* piCoeff, TCoeff* piRefCoeff, UInt uiQp, UInt uiRefQp, Bool& bCoded  )
{
	// DECLARATIONS
	TCoeff cPredCoeff[64];

	// PREDICT THE COEFFICIENTS
	ScaleCoeffLevels( cPredCoeff, piRefCoeff, uiQp, uiRefQp, 64 );

	// ADJUST THE TRANSMITTED COEFFICIENTS
	for( Int n=0; n<64; n++ )
	{
		piCoeff[n] += cPredCoeff[n];
		if( piCoeff[n] )
			bCoded = true;
	}

	return Err::m_nOK;
}

ErrVal Transform::addPredictionChromaBlocks( TCoeff* piCoeff, TCoeff* piRef, UInt uiQp, UInt uiRefQp, Bool& bDCflag, Bool& bACflag )
{

	// DECLARATIONS
	TCoeff cScaledRef[64];

	for( UInt x=0; x<0x40; x+=0x10 )
	{
		ScaleCoeffLevels( &cScaledRef[x], &piRef[x], uiQp, uiRefQp, 16 );

		for( UInt n=0; n<16; n++ )
		{
			piCoeff[x+n] += cScaledRef[x+n];

			if( piCoeff[x+n] )
			{
				if( n%16 )
					bACflag = true;
				else
					bDCflag = true;
			}
		}
	}

	return Err::m_nOK;

}



ErrVal Transform::invTransformChromaBlocks( Pel* puc, Int iStride, TCoeff* piCoeff )
{
  xInvTransform4x4Blk( puc,     iStride, piCoeff + 0x00 );
  xInvTransform4x4Blk( puc + 4, iStride, piCoeff + 0x10 );
  puc += iStride << 2;
  xInvTransform4x4Blk( puc,     iStride, piCoeff + 0x20 );
  xInvTransform4x4Blk( puc + 4, iStride, piCoeff + 0x30 );

  return Err::m_nOK;
}





ErrVal Transform::transform4x4Blk( YuvMbBuffer*              pcOrgData,	// 原始数据
                                   YuvMbBuffer*              pcPelData,				// 预测数据
                                   TCoeff*                      piCoeff,		// 保存量化后系数 一开始为空
                                   const UChar*                 pucScale,
                                   UInt&                        ruiAbsSum  )
{
  TCoeff  aiTemp[64];		// 临时依次保存 变换后系数、反量化后系数
  XPel*   pOrg    = pcOrgData->getLumBlk();
  XPel*   pRec    = pcPelData->getLumBlk();
  Int     iStride = pcPelData->getLStride();

  xForTransform4x4Blk( pOrg, pRec, iStride, aiTemp );
  xQuantDequantUniform4x4( piCoeff, aiTemp, m_cLumaQp, pucScale, ruiAbsSum );

  if( m_storeCoeffFlag ) // store the coefficients
  {
    for( UInt ui=0; ui<16; ui++ )
    {
      piCoeff[ui].setLevel( aiTemp[ui].getCoeff() );
    }
  }

  ROTRS( 0 == ruiAbsSum, Err::m_nOK );
  xInvTransform4x4Blk( pRec, iStride, aiTemp );
  return Err::m_nOK;
}


ErrVal
Transform::transform8x8BlkCGS( YuvMbBuffer* pcOrgData,
                            YuvMbBuffer* pcPelData,
                            TCoeff*         piCoeff,
                            TCoeff*         piCoeffBase,
                            const UChar*    pucScale,
                            UInt&           ruiAbsSum )
{
  TCoeff  aiTemp[64];
  Int normAdjust[] = { 8, 9, 5, 9,   8, 9, 5, 9 };

  XPel*   pOrg    = pcOrgData->getLumBlk();
  XPel*   pRec    = pcPelData->getLumBlk();
  Int     iStride = pcPelData->getLStride();

  xForTransform8x8Blk     ( pOrg, pRec, iStride, aiTemp );
  UInt ui=0;

  // get the baselayer coefficients
  for( ui=0; ui<64; ui++ )
	  aiTemp[ui] = ( aiTemp[ui].getCoeff() - ( ( normAdjust[ui/8]*normAdjust[ui%8]*(Int)piCoeffBase[ui].getLevel() + (1<<5) ) >> 6 ) );

  xQuantDequantUniform8x8 ( piCoeff, aiTemp, m_cLumaQp, pucScale, ruiAbsSum );

  // add the base layer coeff back
  for( ui=0; ui<64; ui++ )
  {
   aiTemp[ui] = piCoeffBase[ui].getLevel() + aiTemp[ui].getCoeff();

    // store the coefficients
   if( m_storeCoeffFlag )
      piCoeff[ui].setLevel( aiTemp[ui].getCoeff() );
  }

  invTransform8x8Blk      ( pRec, iStride, aiTemp );

  return Err::m_nOK;
}

ErrVal Transform::transform4x4BlkCGS( YuvMbBuffer*         pcOrgData,
                                   YuvMbBuffer*            pcPelData,
                                   TCoeff*                    piCoeff,
                                   TCoeff*                    piCoeffBase,                 //参考层的量化后系数
                                   const UChar*               pucScale,
                                   UInt&                      ruiAbsSum )
{
  TCoeff  aiTemp[64];
  Int normAdjust[] = { 4, 5, 4, 5 };
  XPel*   pOrg    = pcOrgData->getLumBlk();            // 原始（未减去BL预测值的）宏块   |   原始（减去BL预测值后）宏块
  XPel*   pRec    = pcPelData->getLumBlk();
  Int     iStride = pcPelData->getLStride();

  xForTransform4x4Blk( pOrg, pRec, iStride, aiTemp );     // aiTemp 是 变换后的系数
  UInt ui=0;

  // get the baselayer coefficients      aiTemp 保存 【当前量化前 】与 【baselayer反量化后系数(修正过)】 的差  ==== 【变换系数差值】
  for( ui=0; ui<16; ui++ )
	  aiTemp[ui] = ( aiTemp[ui].getCoeff() - ( ( normAdjust[ui/4]*normAdjust[ui%4]*(Int)piCoeffBase[ui].getLevel() + (1<<5) ) >> 6 ) );

  xQuantDequantUniform4x4( piCoeff, aiTemp, m_cLumaQp, pucScale, ruiAbsSum );      //  aiTemp【差值】进行量化反量化，   piCoeff---【差值量化后】, aiTemp---【差值量化、反量化后，变回原来的】

  // add the base layer coeff back
  for( ui=0; ui<16; ui++ )
  {
    aiTemp[ui] = piCoeffBase[ui].getLevel() + aiTemp[ui].getCoeff();            //aiTemp =【baselayer反量化后系数(修正过)】  +【差值反量化后】， 得到完整的变换系数值--还是反量化后的

    // store the coefficients
    if( m_storeCoeffFlag )
		piCoeff[ui].setLevel( aiTemp[ui].getCoeff() );              // level保存反量化的值

  }

  xInvTransform4x4Blk( pRec, iStride, aiTemp );           // 反变换  重建

  return Err::m_nOK;
}




Void Transform::xForTransform4x4Blk( XPel* pucOrg, XPel* pucRec, Int iStride, TCoeff* piPredCoeff )
{
  Int aai[4][4];
  Int tmp1, tmp2;

  for( Int y = 0; y < 4; y++ )
  {
    Int ai[4];

    ai[0] = pucOrg[0] - pucRec[0];
    ai[1] = pucOrg[1] - pucRec[1];
    ai[2] = pucOrg[2] - pucRec[2];
    ai[3] = pucOrg[3] - pucRec[3];

    tmp1 = ai[0] + ai[3];
    tmp2 = ai[1] + ai[2];

    aai[0][y] = tmp1 + tmp2;
    aai[2][y] = tmp1 - tmp2;

    tmp1 = ai[0] - ai[3];
    tmp2 = ai[1] - ai[2];

    aai[1][y] = tmp1 * 2 + tmp2 ;
    aai[3][y] = tmp1  - tmp2 * 2;
    pucRec += iStride;
    pucOrg += iStride;
  }


  for( Int x = 0; x < 4; x++, piPredCoeff++ )
  {
    tmp1 = aai[x][0] + aai[x][3];
    tmp2 = aai[x][1] + aai[x][2];

    piPredCoeff[0] = tmp1 + tmp2;
    piPredCoeff[8] = tmp1 - tmp2;

    tmp1 = aai[x][0] - aai[x][3];
    tmp2 = aai[x][1] - aai[x][2];

    piPredCoeff[4]  = tmp1 * 2 + tmp2;
    piPredCoeff[12] = tmp1 - tmp2 * 2;
  }
}



//						puc 传入：预测数据 传出：重建数据=预测+残差
Void Transform::xInvTransform4x4Blk( XPel* puc, Int iStride, TCoeff* piCoeff )
{

  Int aai[4][4];
  Int tmp1, tmp2;
  Int x, y;
  Int iStride2=2*iStride;
  Int iStride3=3*iStride;

  for( x = 0; x < 4; x++, piCoeff+=4 )
  {
    tmp1 = piCoeff[0] + piCoeff[2];
    tmp2 = (piCoeff[3]>>1) + piCoeff[1];

    aai[0][x] = tmp1 + tmp2;
	  aai[3][x] = tmp1 - tmp2;

    tmp1 = piCoeff[0] - piCoeff[2];
    tmp2 = (piCoeff[1]>>1) - piCoeff[3];

    aai[1][x] = tmp1 + tmp2;
    aai[2][x] = tmp1 - tmp2;
  }

  for( y = 0; y < 4; y++, puc ++ )
  {
    tmp1 =  aai[y][0] + aai[y][2];
    tmp2 = (aai[y][3]>>1) + aai[y][1];

    puc[0]        = xClip( xRound( tmp1 + tmp2) + puc[0]        );		// 残差 + 预测值 = 重建值
	  puc[iStride3] = xClip( xRound( tmp1 - tmp2) + puc[iStride3] );

    tmp1 =  aai[y][0] - aai[y][2];
    tmp2 = (aai[y][1]>>1) - aai[y][3];

    puc[iStride]  = xClip( xRound( tmp1 + tmp2) + puc[iStride]  );
	  puc[iStride2] = xClip( xRound( tmp1 - tmp2) + puc[iStride2] );

  }
}
              //          注意：：： pcOrgData内容其实并没有改变，
ErrVal Transform::transformMb16x16( YuvMbBuffer* pcOrgData, YuvMbBuffer* pcPelData, TCoeff* piCoeff, const UChar* pucScale, UInt& ruiDcAbs, UInt& ruiAcAbs )
{
  XPel* pucOrg  = pcOrgData->getMbLumAddr();      // 原始宏块
  XPel* pucRec  = pcPelData->getMbLumAddr();        // 预测的宏块   ===  重建的宏块
  Int   iStride = pcPelData->getLStride();

  TCoeff aiCoeff[256];              //变换系数

  Int x, n;
  Int iOffset = 0;

  for( n = 0; n < 16; n+=4 )     //16个 4x4     n = 0  4  8  12
  {
    for( x = 0; x < 4; x++ ) 
    {
      UInt uiBlk = x+n;            // （0,1,2,3）   (4,5,6,7)  (8,9,10,11) ....
      Int iOffsetBlk = iOffset + (x << 2);           //  (0,4,8,12)    ( 96,100,104,108)    
      xForTransform4x4Blk( pucOrg + iOffsetBlk, pucRec + iOffsetBlk, iStride, &aiCoeff[uiBlk<<4] );
    }
    iOffset += iStride << 2;     //下移四行
  }
  //至此  所有4x4变换结束  
  xForTransformLumaDc( aiCoeff );           //对DC系数进行4x4哈达玛变换

  for( n = 0; n < 16; n ++ )              //对每个块进行量化和反量化
  {                                                               //          量化后值           反量化后值          Qp            scalinglist     DC和          AC和
    xQuantDequantNonUniformLuma( &piCoeff[n<<4], &aiCoeff[n<<4], m_cLumaQp, pucScale, ruiDcAbs, ruiAcAbs );
  }

  for( n = 0; n < 16; n ++ )
  {
    aiCoeff[n<<4] = piCoeff[n<<4];              //DC系数反量化值先用量化值代替
  }

  Int iQpScale = ( g_aaiDequantCoef[m_cLumaQp.rem()][0] << m_cLumaQp.per() );     //DC系数量化值
  if( pucScale )
  {
    iQpScale = ( iQpScale * pucScale[0] ) >> 4;
  }

  invTransformDcCoeff( aiCoeff, iQpScale );             //反哈达玛变换

  iOffset = 0;
  for( n = 0; n < 16; n += 4 )
  {
    for( x = 0; x < 4; x++ )
    {
      UInt uiBlk = x+n;
      Int iOffsetBlk = iOffset + (x << 2);
      xInvTransform4x4Blk( pucRec + iOffsetBlk, iStride, &aiCoeff[uiBlk<<4] );                 //对每个4x4块进行反DCT变换   重建值写入pucRec  （pcPelData）
    }
    iOffset += iStride << 2;
  }

  for( x = 0; x < 256; x++ )
  {
    piCoeff[x].setLevel( aiCoeff[x].getCoeff() );     //把反量化后的系数存入 m_iLevelValue  
  }

  return Err::m_nOK;
}

ErrVal Transform::transformChromaBlocks( XPel*          pucOrg,
                                         XPel*          pucRec,
                                         const CIdx     cCIdx,
                                         Int            iStride,
                                         TCoeff*        piCoeff,             //量化后
                                         TCoeff*        piQuantCoeff,  //反量化后
                                         const UChar*   pucScale,
                                         UInt&          ruiDcAbs,
                                         UInt&          ruiAcAbs )
{
  Int iOffset = 0;

  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x00);
  iOffset += 4;
  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x10);
  iOffset  = 4*iStride;
  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x20);
  iOffset += 4;
  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x30);

  xForTransformChromaDc( piQuantCoeff );

  const QpParameter& rcCQp  =  ( cCIdx.plane() ? m_cCrQp : m_cCbQp );
  xQuantDequantNonUniformChroma( piCoeff + 0x00, piQuantCoeff + 0x00, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );
  xQuantDequantNonUniformChroma( piCoeff + 0x10, piQuantCoeff + 0x10, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );
  xQuantDequantNonUniformChroma( piCoeff + 0x20, piQuantCoeff + 0x20, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );
  xQuantDequantNonUniformChroma( piCoeff + 0x30, piQuantCoeff + 0x30, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );

  if( m_storeCoeffFlag )
  {
    for( UInt ui=0; ui<64; ui++ )
    {
      piCoeff[ui].setLevel( piQuantCoeff[ui].getCoeff() );  // store the dequantized coeffs in TCoeff.level
    }
  }

  return Err::m_nOK;
}

ErrVal Transform::transformChromaBlocksCGS( XPel*       pucOrg,
                                         XPel*          pucRec,
                                         const CIdx     cCIdx,
                                         Int            iStride,
                                         TCoeff*        piCoeff,
                                         TCoeff*        piQuantCoeff,
                                         TCoeff*        piCoeffBase,
                                         const UChar*   pucScale,
                                         UInt&          ruiDcAbs,
                                         UInt&          ruiAcAbs )
{
  Int iOffset = 0;
  Int normAdjust[] = { 4, 5, 4, 5};

  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x00);
  iOffset += 4;
  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x10);
  iOffset  = 4*iStride;
  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x20);
  iOffset += 4;
  xForTransform4x4Blk( pucOrg + iOffset, pucRec + iOffset, iStride, piQuantCoeff + 0x30);

  xForTransformChromaDc( piQuantCoeff );

  // substract the baselayer coefficients
  for( UInt uiOffset = 0; uiOffset<0x40; uiOffset+=0x10 )
  {
    piQuantCoeff[uiOffset+0] = ( piQuantCoeff[uiOffset+0].getCoeff() - ( ( piCoeffBase[uiOffset+0].getLevel() + (1<<4) ) >> 5 ) );
	  for( UInt ui=1; ui<16; ui++ )
		  piQuantCoeff[uiOffset+ui] = ( piQuantCoeff[uiOffset+ui].getCoeff() - ( ( normAdjust[ui/4] * normAdjust[ui%4] * piCoeffBase[uiOffset+ui].getLevel() + (1<<5) ) >> 6 ) );
  }

  const QpParameter&  rcCQp  = ( cCIdx.plane() ? m_cCrQp : m_cCbQp );
  xQuantDequantNonUniformChroma( piCoeff + 0x00, piQuantCoeff + 0x00, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );
  xQuantDequantNonUniformChroma( piCoeff + 0x10, piQuantCoeff + 0x10, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );
  xQuantDequantNonUniformChroma( piCoeff + 0x20, piQuantCoeff + 0x20, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );
  xQuantDequantNonUniformChroma( piCoeff + 0x30, piQuantCoeff + 0x30, rcCQp, pucScale, ruiDcAbs, ruiAcAbs );

  // add the base layer coeff back and also store the dequantized coeffs
  for( UInt ui=0; ui<64; ui++ )
  {
    piQuantCoeff[ui] = piCoeffBase[ui].getLevel() + piQuantCoeff[ui].getCoeff();
    if( m_storeCoeffFlag )
      piCoeff[ui].setLevel( piQuantCoeff[ui].getCoeff() );
  }

  return Err::m_nOK;
}


ErrVal Transform::invTransformChromaBlocks( XPel* puc, Int iStride, TCoeff* piCoeff )
{
  xInvTransform4x4Blk( puc,     iStride, piCoeff + 0x00 );
  xInvTransform4x4Blk( puc + 4, iStride, piCoeff + 0x10 );
  puc += iStride << 2;
  xInvTransform4x4Blk( puc,     iStride, piCoeff + 0x20 );
  xInvTransform4x4Blk( puc + 4, iStride, piCoeff + 0x30 );

  return Err::m_nOK;
}


ErrVal Transform::invTransform4x4Blk( XPel* puc, Int iStride, TCoeff* piCoeff )
{
  xInvTransform4x4Blk( puc, iStride, piCoeff );

  return Err::m_nOK;
}





ErrVal
Transform::transform8x8Blk( YuvMbBuffer* pcOrgData,
                            YuvMbBuffer* pcPelData,
                            TCoeff*         piCoeff,
                            const UChar*    pucScale,
                            UInt&           ruiAbsSum )
{
  TCoeff  aiTemp[64];
  XPel*   pOrg    = pcOrgData->getLumBlk();
  XPel*   pRec    = pcPelData->getLumBlk();
  Int     iStride = pcPelData->getLStride();

  xForTransform8x8Blk     ( pOrg, pRec, iStride, aiTemp );
  xQuantDequantUniform8x8 ( piCoeff, aiTemp, m_cLumaQp, pucScale, ruiAbsSum );

  if( m_storeCoeffFlag )
  {
    for( UInt ui=0; ui<64; ui++ )
      piCoeff[ui].setLevel( aiTemp[ui].getCoeff() );  // store the dequantized coeffs are stored in TCoeff.level
  }

  invTransform8x8Blk      ( pRec, iStride, aiTemp );

  return Err::m_nOK;
}



ErrVal
Transform::invTransform8x8Blk( XPel*    puc,
                               Int      iStride,
                               TCoeff*  piCoeff )
{
  Int aai[8][8];
  Int n;

  for( n = 0; n < 8; n++ )
  {
    TCoeff* pi = piCoeff + n*8;
    Int     ai1[8];
    Int     ai2[8];

    ai1[0] = pi[0] + pi[4];
    ai1[2] = pi[0] - pi[4];

    ai1[4] = (pi[2]>>1) -  pi[6];
    ai1[6] =  pi[2]     + (pi[6]>>1);

    ai1[1] = pi[5] - pi[3] - pi[7] - (pi[7]>>1);
    ai1[3] = pi[1] + pi[7] - pi[3] - (pi[3]>>1);;
    ai1[5] = pi[7] - pi[1] + pi[5] + (pi[5]>>1);
    ai1[7] = pi[3] + pi[5] + pi[1] + (pi[1]>>1);

    ai2[0] = ai1[0] + ai1[6];
    ai2[6] = ai1[0] - ai1[6];

    ai2[2] = ai1[2] + ai1[4];
    ai2[4] = ai1[2] - ai1[4];

    ai2[1] = ai1[1] + (ai1[7]>>2);
    ai2[7] = ai1[7] - (ai1[1]>>2);

    ai2[3] =  ai1[3]     + (ai1[5]>>2);
    ai2[5] = (ai1[3]>>2) -  ai1[5];

    aai[n][0] = ai2[0] + ai2[7];
    aai[n][1] = ai2[2] + ai2[5];
    aai[n][2] = ai2[4] + ai2[3];
    aai[n][3] = ai2[6] + ai2[1];
    aai[n][4] = ai2[6] - ai2[1];
    aai[n][5] = ai2[4] - ai2[3];
    aai[n][6] = ai2[2] - ai2[5];
    aai[n][7] = ai2[0] - ai2[7];
  }

  for( n = 0; n < 8; n++, puc++ )
  {
    Int ai1[8];
    Int ai2[8];

    ai1[0] =  aai[0][n]     +  aai[4][n];
    ai1[1] =  aai[5][n]     -  aai[3][n]     - aai[7][n] - (aai[7][n]>>1);
    ai1[2] =  aai[0][n]     -  aai[4][n];
    ai1[3] =  aai[1][n]     +  aai[7][n]     - aai[3][n] - (aai[3][n]>>1);
    ai1[4] = (aai[2][n]>>1) -  aai[6][n];
    ai1[5] =  aai[7][n]     -  aai[1][n]     + aai[5][n] + (aai[5][n]>>1);
    ai1[6] =  aai[2][n]     + (aai[6][n]>>1);
    ai1[7] =  aai[3][n]     +  aai[5][n]     + aai[1][n] + (aai[1][n]>>1);

    ai2[2] = ai1[2] + ai1[4];
    ai2[4] = ai1[2] - ai1[4];

    ai2[0] = ai1[0] + ai1[6];
    ai2[6] = ai1[0] - ai1[6];

    ai2[1] = ai1[1] + (ai1[7]>>2);
    ai2[7] = ai1[7] - (ai1[1]>>2);

    ai2[3] =  ai1[3]     + (ai1[5]>>2);
    ai2[5] = (ai1[3]>>2) -  ai1[5];

    puc[0*iStride] = xClip( xRound( ai2[0] + ai2[7] ) + puc[0*iStride] );
    puc[1*iStride] = xClip( xRound( ai2[2] + ai2[5] ) + puc[1*iStride] );
    puc[2*iStride] = xClip( xRound( ai2[4] + ai2[3] ) + puc[2*iStride] );
    puc[3*iStride] = xClip( xRound( ai2[6] + ai2[1] ) + puc[3*iStride] );
    puc[4*iStride] = xClip( xRound( ai2[6] - ai2[1] ) + puc[4*iStride] );
    puc[5*iStride] = xClip( xRound( ai2[4] - ai2[3] ) + puc[5*iStride] );
    puc[6*iStride] = xClip( xRound( ai2[2] - ai2[5] ) + puc[6*iStride] );
    puc[7*iStride] = xClip( xRound( ai2[0] - ai2[7] ) + puc[7*iStride] );
  }

  return Err::m_nOK;
}


Void
Transform::x4x4DequantChroma( TCoeff*             piQCoeff,
                              TCoeff*             piCoeff,
                              const QpParameter&  rcQp,
                              const UChar*        pucScale )
{
  Int iAdd = ( ( 1 << 3 ) >> rcQp.per() ) << rcQp.per();

  piCoeff[0] = piQCoeff[0] * ( pucScale ? pucScale[0] : 16 ) * ( g_aaiDequantCoef[rcQp.rem()][0] << rcQp.per() );

  for( Int n = 1; n < 16; n++ )
  {
    if( piQCoeff[n] )
    {
      piCoeff[n]    = piQCoeff[n] * ( g_aaiDequantCoef[rcQp.rem()][n] << rcQp.per() );
      if( pucScale )
      {
        piCoeff[n]  = ( piCoeff[n] * pucScale[n] + iAdd ) >> 4;
      }
    }
  }
}


Void
Transform::xForTransform8x8Blk( XPel* pucOrg, XPel* pucRec, Int iStride, TCoeff* piPredCoeff )
{
  Int aai[8][8];

  for( Int i = 0; i < 8; i++, pucOrg += iStride, pucRec += iStride)
  {
    Int ai  [8];
    Int ai1 [8];
    Int ai2 [8];

    ai[0] = pucOrg[0] - pucRec[0];
    ai[1] = pucOrg[1] - pucRec[1];
    ai[2] = pucOrg[2] - pucRec[2];
    ai[3] = pucOrg[3] - pucRec[3];
    ai[4] = pucOrg[4] - pucRec[4];
    ai[5] = pucOrg[5] - pucRec[5];
    ai[6] = pucOrg[6] - pucRec[6];
    ai[7] = pucOrg[7] - pucRec[7];

    ai1[0] = ai[0] + ai[7];
    ai1[1] = ai[1] + ai[6];
    ai1[2] = ai[2] + ai[5];
    ai1[3] = ai[3] + ai[4];

    ai1[4] = ai[0] - ai[7];
    ai1[5] = ai[1] - ai[6];
    ai1[6] = ai[2] - ai[5];
    ai1[7] = ai[3] - ai[4];

    ai2[0] = ai1[0] + ai1[3];
    ai2[1] = ai1[1] + ai1[2];
    ai2[2] = ai1[0] - ai1[3];
    ai2[3] = ai1[1] - ai1[2];
    ai2[4] = ai1[5] + ai1[6] + ((ai1[4]>>1) + ai1[4]);
    ai2[5] = ai1[4] - ai1[7] - ((ai1[6]>>1) + ai1[6]);
    ai2[6] = ai1[4] + ai1[7] - ((ai1[5]>>1) + ai1[5]);
    ai2[7] = ai1[5] - ai1[6] + ((ai1[7]>>1) + ai1[7]);

    aai[0][i] =  ai2[0]     +  ai2[1];
    aai[2][i] =  ai2[2]     + (ai2[3]>>1);
    aai[4][i] =  ai2[0]     -  ai2[1];
    aai[6][i] = (ai2[2]>>1) -  ai2[3];

    aai[1][i] =  ai2[4]     + (ai2[7]>>2);
    aai[3][i] =  ai2[5]     + (ai2[6]>>2);
    aai[5][i] =  ai2[6]     - (ai2[5]>>2);
    aai[7][i] = (ai2[4]>>2) -  ai2[7];
  }

  // vertical transform
  for( Int n = 0; n < 8; n++, piPredCoeff++)
  {
    Int ai1[8];
    Int ai2[8];

    ai1[0] = aai[n][0] + aai[n][7];
    ai1[1] = aai[n][1] + aai[n][6];
    ai1[2] = aai[n][2] + aai[n][5];
    ai1[3] = aai[n][3] + aai[n][4];
    ai1[4] = aai[n][0] - aai[n][7];
    ai1[5] = aai[n][1] - aai[n][6];
    ai1[6] = aai[n][2] - aai[n][5];
    ai1[7] = aai[n][3] - aai[n][4];

    ai2[0] = ai1[0] + ai1[3];
    ai2[1] = ai1[1] + ai1[2];
    ai2[2] = ai1[0] - ai1[3];
    ai2[3] = ai1[1] - ai1[2];
    ai2[4] = ai1[5] + ai1[6] + ((ai1[4]>>1) + ai1[4]);
    ai2[5] = ai1[4] - ai1[7] - ((ai1[6]>>1) + ai1[6]);
    ai2[6] = ai1[4] + ai1[7] - ((ai1[5]>>1) + ai1[5]);
    ai2[7] = ai1[5] - ai1[6] + ((ai1[7]>>1) + ai1[7]);

    piPredCoeff[ 0] =  ai2[0]     +  ai2[1];
    piPredCoeff[16] =  ai2[2]     + (ai2[3]>>1);
    piPredCoeff[32] =  ai2[0]     -  ai2[1];
    piPredCoeff[48] = (ai2[2]>>1) -  ai2[3];

    piPredCoeff[ 8] =  ai2[4]     + (ai2[7]>>2);
    piPredCoeff[24] =  ai2[5]     + (ai2[6]>>2);
    piPredCoeff[40] =  ai2[6]     - (ai2[5]>>2);
    piPredCoeff[56] = (ai2[4]>>2) -  ai2[7];
  }
}



Void
Transform::xQuantDequantUniform8x8( TCoeff*             piQCoeff,
                                    TCoeff*             piCoeff,
                                    const QpParameter&  rcQp,
                                    const UChar*        pucScale,
                                    UInt&               ruiAbsSum )
{
  UInt  uiAbsSum  = 0;
  Int   iAdd      = ( 1 << 5 ) >> rcQp.per();

  for( Int n = 0; n < 64; n++ )
  {
    Int iLevel  = piCoeff[n];
    Int iSign   = iLevel;

    iLevel      = abs( iLevel ) * g_aaiQuantCoef64[ rcQp.rem() ][ n ];
    if( pucScale )
    {
      iLevel    = ( iLevel << 4 ) / pucScale[ n ];
    }
    iLevel      = ( iLevel + 2*rcQp.add() ) >> ( rcQp.bits() + 1 );

    if( 0 != iLevel )
    {
      iSign      >>= 31;
      Int iDeScale = g_aaiDequantCoef64[ rcQp.rem() ][ n ];
      uiAbsSum    += iLevel;
      iLevel      ^= iSign;
      iLevel      -= iSign;
      piQCoeff[n]  = iLevel;

      if( pucScale )
      {
        piCoeff[n]   = ( (iLevel*iDeScale*pucScale[n] + iAdd) << rcQp.per() ) >> 6;
      }
      else
      {
        piCoeff[n]   = ( (iLevel*iDeScale*16          + iAdd) << rcQp.per() ) >> 6;
      }
    }
    else
    {
      piQCoeff[n] = 0;
      piCoeff [n] = 0;
    }
  }

  ruiAbsSum   = uiAbsSum;
}


// h264 namepace end
H264AVC_NAMESPACE_END
