/*******************************************************************
 * 简介：       通过ReaderWriterQueue队列实现的缓冲区，线程安全
 * 名称：       DataCache.h、DataCache.cpp
 * 创建时间：    2019年12月6号
 * 修改时间：    2019年12月9号      完善代码逻辑
 * *****************************************************************/
#pragma once

#include <vector>
#include "readerwriterqueue.h"
#include <exception>
#include "CommonDefine.h"

namespace NS_DataCache
{
	using namespace moodycamel;

#define DATA_CACHE_MAX_SIZE 100              //一个缓冲区最多有100条队列，可更改
#define DATA_CACHE_QUEUE_MAX_SIZE 500        //一个队列中最大缓存500条数据，可更改

	//缓冲区
	template<class T>
	class DataCache
	{
	private:
		std::vector<BlockingReaderWriterQueue<T>>* m_pDataCache;//为了处理方便，使用vector

	public:
		//构造函数
		DataCache()
		{
			m_pDataCache = new std::vector<BlockingReaderWriterQueue<T>>();
		}
		//析构函数
		~DataCache()
		{
			if (nullptr != m_pDataCache)
			{
				delete m_pDataCache;
				m_pDataCache = nullptr;
			}
		}
		//获取缓冲区大小
		int32_t GetSize()
		{
			return m_pDataCache->size();
		}
		//向缓冲区中加入一个队列,返回这个队列在缓冲区中的索引
		int32_t ExpandDataCacheSize(int32_t& iIndex)
		{
			//扩容，容量校验
			if (m_pDataCache->size() < DATA_CACHE_MAX_SIZE)
			{
				try
				{
					iIndex = m_pDataCache->size();
					m_pDataCache->resize(iIndex + 1);
				}
				catch (std::exception e)
				{
					printf("ERROR：缓冲区扩容失败，%s\n",e.what());
					return RET_ERROR;
				}
			}
			else
			{
				printf("ERROR：缓冲区扩容失败，超过最大缓冲区限制！\n");
			}
			return RET_SUCCESS;
		}
		//通过Index向缓冲区入队一帧数据
		int32_t InsertDataByIndex(T inputValue, int32_t i32BufferIndex)//由于每个线程的Index不同，线程安全
		{
			//入参校验
			if (i32BufferIndex < 0 || i32BufferIndex >= m_pDataCache->size())
			{
				printf("ERROR：向缓冲区存放数据失败，入参校验未通过!\n");
				return RET_ERROR;
			}
			if (!m_pDataCache->at(i32BufferIndex).enqueue(inputValue))//入队
			{
				printf("ERROR：向缓冲区存放数据失败，向第%d个队列中入队时失败！\n", i32BufferIndex);
				return RET_ERROR;
			}
			return RET_SUCCESS;
		}
		//通过Index缓冲区出队一帧数据，内存由本函数申请
		int32_t GetDataByIndex(int32_t i32BufferIndex, T &outputValue)
		{
			//入参校验
			if (i32BufferIndex < 0 || i32BufferIndex >= m_pDataCache->size())
			{
				printf("ERROR：从缓冲区获取数据失败，入参校验未通过!\n");
				return RET_ERROR;
			}
			//出队
			try
			{
				m_pDataCache->at(i32BufferIndex).wait_dequeue(outputValue);//出队,等待出队，ReaderWriteQueue中的机制，可以实现等待队列中存在完整数据后再出队，否则阻塞
			}
			catch(...)
			{
				printf("ERROR：从缓冲区获取数据失败，出队失败!\n");
				return RET_ERROR;
			}
			return RET_SUCCESS;
		}
	};
};//namespace DataCache