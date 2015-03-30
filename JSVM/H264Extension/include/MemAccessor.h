
#if !defined(AFX_MEMACCESSOR_H__F7B8E14C_79CB_4DAF_A1BE_9AA64D4A2DDA__INCLUDED_)
#define AFX_MEMACCESSOR_H__F7B8E14C_79CB_4DAF_A1BE_9AA64D4A2DDA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <list>

template< class T >
class MemAccessor
{
public:        //默认构造函数  带参数的构造函数  复制构造函数   析构函数
  MemAccessor() :
    m_pcT( NULL ), m_pcOrigT( NULL ), m_uiSize( 0 ), m_uiUsableSize( 0 ) {}

  MemAccessor( T* pcT, UInt uiSize, T* pcOrigT=NULL, UInt uiUsableSize=0 ) :
    m_pcT( pcT ), m_pcOrigT( pcOrigT ), m_uiSize( uiSize ), m_uiUsableSize( uiUsableSize )
  {
    if( NULL == m_pcOrigT ) m_pcOrigT = m_pcT;           //如果是默认参数，则设置成跟前两个参数一样的值
    if( 0 == m_uiUsableSize ) m_uiUsableSize = uiSize;
  }

  MemAccessor( const MemAccessor< T >& rcMemAccessor ) :            //类定义体内可以使用该类对象的private成员
    m_pcT( rcMemAccessor.m_pcT ), m_pcOrigT( rcMemAccessor.m_pcOrigT ), m_uiSize( rcMemAccessor.m_uiSize ) , m_uiUsableSize( rcMemAccessor.m_uiUsableSize ) {}

  virtual ~MemAccessor() {}

public:
  MemAccessor< T >& operator=( const MemAccessor< T >& rcMemAccessor )
  {
    m_pcT = rcMemAccessor.m_pcT;
    m_pcOrigT = rcMemAccessor.m_pcOrigT;
    m_uiSize = rcMemAccessor.m_uiSize;
    m_uiUsableSize = rcMemAccessor.m_uiUsableSize;

    return *this;
  }

public:
  Void set( const MemAccessor< T >& rcMemAccessor )
  {
    m_pcT = rcMemAccessor.m_pcT;
    m_pcOrigT = rcMemAccessor.m_pcOrigT;
    m_uiSize = rcMemAccessor.m_uiSize;
    m_uiUsableSize = rcMemAccessor.m_uiUsableSize;
  }

  Void set( T* pcT, UInt uiSize, T* pcOrigT=NULL, UInt uiUsableSize=0 )
  {
    m_pcT = pcT;
    m_uiSize = uiSize;
    m_pcOrigT = pcOrigT;
    m_uiUsableSize = uiUsableSize;
    if( NULL == pcOrigT ) { m_pcOrigT = pcT; }
    if( 0 == uiUsableSize ) { m_uiUsableSize = m_uiSize; }
  }

  Void clear()
  {
    m_pcT = NULL;
    m_pcOrigT = NULL;
    m_uiSize = 0;
    m_uiUsableSize = 0;
  }
                                                  //没有对应的new 这样delete不安全 除非每次在生成对象前new了m_pcOrigT的空间
  Void deleteData() { if( m_pcOrigT ) { delete[] m_pcOrigT; } m_pcOrigT = NULL; m_pcT = NULL; m_uiSize = 0; m_uiUsableSize = 0; }

  T* data() const { return m_pcT; }
  T* origData() const { return m_pcOrigT; }
  UInt size() const { return m_uiSize; }
  UInt usableSize() const { return m_uiUsableSize; }
  UInt byteSize() const { return sizeof( T ) * m_uiSize; }
  UInt usableByteSize() const { return sizeof( T ) * m_uiUsableSize; }

public:
  ErrVal increasePos( UInt uiPos )          //向后移动当前位置
  {
    ROT( uiPos >= m_uiSize );
    m_pcT += uiPos;
    m_uiSize -= uiPos;
    return Err::m_nOK;
  }


  ErrVal decreasePos( UInt uiPos )          //向前移动当前位置
  {
    ROT( m_pcT - uiPos < m_pcOrigT );
    m_pcT -= uiPos;
    m_uiSize += uiPos;
    return Err::m_nOK;
  }

  ErrVal resetPos()                                 //当前位置回到起点位置
  {
    m_uiSize += m_pcT - m_pcOrigT;
    m_pcT = m_pcOrigT;
    return Err::m_nOK;
  }

  ErrVal increaseEndPos( UInt uiPos )         //增大m_uiSize
  {
    ROT( uiPos > (m_uiUsableSize - m_uiSize) );
    m_uiSize += uiPos;
    return Err::m_nOK;
  }

  ErrVal decreaseEndPos( UInt uiPos )         //减小m_uiSize
  {
    ROT( uiPos > m_uiSize );
    m_uiSize -= uiPos;
    return Err::m_nOK;
  }

  ErrVal resetEndPos()                               //重置末端位置到最后
  {
    m_uiSize = m_uiUsableSize - (m_pcT - m_pcOrigT);
    return Err::m_nOK;
  }

private:
  T* m_pcT;                       //当前位置
  T* m_pcOrigT;                //首地址
  UInt m_uiSize;                 //当前位置距末尾长度
  UInt m_uiUsableSize;       //总长度
};



template< class T >
class MemAccessList
{
public:
  MemAccessList() : m_uiSize( 0 ) {}

  MemAccessList( const MemAccessList< T >& rcMemAccessList ) : m_uiSize( 0 )
  {
    m_uiSize = rcMemAccessList.m_uiSize;
    typename MemAccessorList::const_iterator pcPair;         //告诉编译器这是个类型
    for( pcPair = rcMemAccessList.m_cMemAccessorList.begin(); pcPair != rcMemAccessList.m_cMemAccessorList.end(); pcPair++  )
    {                           //调用push_back往list里添加成员，  参数为类型的构造函数
      m_cMemAccessorList.push_back( MemAccessor< T >( pcPair->data(), pcPair->size(), pcPair->origData(), pcPair->usableSize() ) );
    }
  }

  MemAccessList( T* pcT, UInt uiSize, T* pcOrigT=NULL, UInt uiUsableSize=0 ) : m_uiSize( 0 )
  {
    if( NULL == pcOrigT ) { pcOrigT = pcT; }
    if( 0 == uiUsableSize ) { uiUsableSize = uiSize; }
    push_back( pcT, uiSize, pcOrigT, uiUsableSize );
  }

  virtual ~MemAccessList() {}

  MemAccessList< T >& operator=( const MemAccessList< T >& rcMemAccessList )
  {
    // paranoia
    ROTRS( this == &rcMemAccessList, *this );

    // reset class
    reset();

    m_uiSize = rcMemAccessList.m_uiSize;
    typename MemAccessorList::const_iterator pcPair;
    for( pcPair = rcMemAccessList.m_cMemAccessorList.begin(); pcPair != rcMemAccessList.m_cMemAccessorList.end(); pcPair++  )
    {
      m_cMemAccessorList.push_back( MemAccessor< T >( pcPair->data(), pcPair->size(), pcPair->origData(), pcPair->usableSize() ) );
    }

    return *this;
  }

public:
	//set()操作会把队列清空  然后添加一项
  Void set( T* pcT, UInt uiSize, T* pcOrigT=NULL, UInt uiUsableSize=0 )
  {
    reset();
    if( NULL == pcOrigT ) { pcOrigT = pcT; }
    if( 0 == uiUsableSize ) { uiUsableSize = uiSize; }
    push_back( pcT, uiSize, pcOrigT, uiUsableSize );
  }

  Void set( MemAccessor< T >& rcMemAccessor )
  {
    reset();
    push_back( rcMemAccessor );
  }

  Void copyPayload( T*& rpcT, UInt& ruiSize )     //把当前MemAccessList类中的list的每个成员拷贝到 T*& rpcT 中，总长度 ruiSize
  {
    if( 0 == m_uiSize )           //list为空
    {
      rpcT = NULL;
      ruiSize = 0;
      return;
    }

    ruiSize = m_uiSize;
    rpcT = new T[ruiSize];             //先new了足够空间才能使用memcpy()函数
    ROTV( NULL == rpcT );

    UInt uiPos = 0;
    MemAccessorListIterator pcPair;
    for( pcPair = m_cMemAccessorList.begin(); pcPair != m_cMemAccessorList.end(); pcPair++ )
    {
      ROTV( uiPos + pcPair->size() > ruiSize );
      //memcpy( &rpcT[uiPos], pcPair->data(), sizeof(T) * pcPair->size() );     用迭代器调用的MemAccessor<T>的size()函数返回m_uiSize   直接调用byteSize()更好
      memcpy( &rpcT[uiPos], pcPair->data(), pcPair->byteSize() );              //***********************我修改的
	  uiPos += pcPair->size();
    }

  }

  UInt listSize() const { return m_cMemAccessorList.size(); }

  T* entryData( UInt uiEntryPos )       //读取第uiEntryPos个条目的data()
  {
    // check if we have more than one entry
    ROTR( uiEntryPos >= m_cMemAccessorList.size(), NULL );
    typename MemAccessorList::const_iterator pcMemAccessor;
    for( pcMemAccessor = m_cMemAccessorList.begin(); uiEntryPos-- != 0; pcMemAccessor++ )
      ;
    return pcMemAccessor->data();
  }

  UInt entrySize( UInt uiEntryPos )       //读取第uiEntryPos个条目的size()
  {
    // check if we have more than one entry
    ROTR( uiEntryPos >= m_cMemAccessorList.size(), 0 );
    typename MemAccessorList::const_iterator pcMemAccessor;
    for( pcMemAccessor = m_cMemAccessorList.begin(); uiEntryPos-- != 0; pcMemAccessor++ )
      ;
    return pcMemAccessor->size();
  }

  UInt entryByteSize( UInt uiEntryPos ) const { return sizeof( T ) * entrySize( uiEntryPos ); }
  UInt size() const { return m_uiSize; }
  UInt byteSize() const { return m_uiSize * sizeof( T ); }

public:
  Void reset() { m_cMemAccessorList.clear(); m_uiSize = 0; }          //调用list的clear()  清空队列

public:
  Void push_back( T* pcT, UInt uiSize, T* pcOrigT=NULL, UInt uiUsableSize=0 )         //往list尾部添加成员
  {
    if( NULL == pcOrigT ) { pcOrigT = pcT; }
    if( 0 == uiUsableSize ) { uiUsableSize = uiSize; }
    m_cMemAccessorList.push_back( MemAccessor< T >( pcT, uiSize, pcOrigT, uiUsableSize ) );
    m_uiSize += uiSize;
  }

  Void push_back( MemAccessor< T >& rcMemAccessor )
  {
    m_cMemAccessorList.push_back( rcMemAccessor );
	m_uiSize += rcMemAccessor.m_uiSize; 
  }

  Void push_front( T* pcT, UInt uiSize, T* pcOrigT=NULL, UInt uiUsableSize=0 )           //往list头部添加成员
  {
    if( NULL == pcOrigT ) { pcOrigT = pcT; }
    if( 0 == uiUsableSize ) { uiUsableSize = uiSize; }
    m_cMemAccessorList.push_front( MemAccessor< T >( pcT, uiSize, pcOrigT, uiUsableSize ) );
    m_uiSize += uiSize;
  }

  Void push_front( MemAccessor< T >& rcMemAccessor )
  {
    m_cMemAccessorList.push_front( rcMemAccessor );
	m_uiSize += rcMemAccessor.m_uiSize;
  }

private:
  typedef std::list< MemAccessor< T > > MemAccessorList;
  typedef typename MemAccessorList::iterator MemAccessorListIterator;

private:
  MemAccessorList m_cMemAccessorList;
  UInt m_uiSize;
};



#endif // !defined(AFX_MEMACCESSOR_H__F7B8E14C_79CB_4DAF_A1BE_9AA64D4A2DDA__INCLUDED_)
