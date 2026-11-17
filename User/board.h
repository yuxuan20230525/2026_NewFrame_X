#ifndef __DEFINE__
#define __DEFINE__

#define IF_USE_DJIC 1
#define IF_USE_V5_V5PLUS_V6 0

#define USE_JUDGEMENT 0 // 是否使用裁判系统串口
#define USE_TEMP_CTRL 1// 是否使用imu温控
#define USE_RC 0        // 是否使用遥控器
#define USE_POWER  0 // 是否使用功率
#define BMI_NO_Init 0   // 复位后不初始化,加快复位时间
#define USE_VISION 0   // 是否使用视觉串口


#if IF_USE_DJIC        																												//c板专属功能
#define USE_BUZZER 1    // 是否使用蜂鸣器

#if USE_BUZZER                     //io冲突功能
#define USE_DJIC 1                 //使用板载蜂鸣器
#define USE_OTHER 0							 //使用外部蜂鸣器     TIM8   CH3   PI7
#endif

#define USE_USB_VISION 1 // 使用usb虚拟串口当作视觉串口，只有c板可以使用
#define USE_LOG 0  // 使用车载log系统（默认使用c板的usb串口发送，兼容视觉串口。）
#endif


#if !USE_JUDGEMENT                     //io冲突功能
#define USE_RCVT03 1
#define USE_Judgement_Vofa 0
#endif

#if !USE_VISION	
#define USE_VISION_Vofa 0
#endif

#define USE_CAMERATM 1


// 已经自动修改 HSE_VALUE   PLL_M
#endif
#ifndef STM32F4_MASK // 在stm32f4xx.h中屏蔽
#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/***************硬件中断分组******************/
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stdlib.h"
#include "gpio.h"
// DSP库
#include "arm_math.h"
#include "TDT_User.h"

#include "systick.h"
#include "timer.h"

#ifdef __cplusplus // 声明c++库的时候写这个，在.c文件中自动屏蔽
#include <vector>
#include <functional>
#include <queue>
#include <initializer_list>
#include <algorithm>
#endif

#ifdef __cplusplus
template <typename T, std::size_t DefaultSize = 6>
class myvector
{
public:
    // 默认构造函数，使用默认最大容量
    myvector() : sz(0) {}

    // 迭代器范围构造函数
    myvector(const T *first, const T *last) : sz(0)
    {
        sz = last - first;
        if (sz > DefaultSize)
        {
            sz = DefaultSize; // 限制有效元素数量不超过 DefaultSize
        }
        for (std::size_t i = 0; i < sz; ++i)
        {
            data[i] = first[i];
        }
    }

    // 拷贝构造函数
    myvector(const myvector &other) : sz(other.sz)
    {

        if (sz > DefaultSize)
        {
            sz = DefaultSize; // 限制有效元素数量不超过 DefaultSize
        }
        for (std::size_t i = 0; i < sz; ++i)
        {
            data[i] = other.data[i];
        }
    }
    // 赋值操作符重载
    myvector &operator=(const myvector &other)
    {
        if (this != &other)
        {
            sz = other.sz;
            if (sz > DefaultSize)
            {
                sz = DefaultSize; // 限制有效元素数量不超过 DefaultSize
            }
            for (std::size_t i = 0; i < sz; ++i)
            {
                data[i] = other.data[i];
            }
        }
        return *this;
    }

    // 使用初始化列表进行赋值
    myvector &operator=(std::initializer_list<T> init)
    {
        clear(); // 清空当前数据
        sz = init.size();
        if (sz > DefaultSize)
        {
            sz = DefaultSize; // 限制大小不超过 DefaultSize
        }
        std::copy(init.begin(), init.begin() + sz, data); // 复制初始化列表数据到数组
        return *this;
    }

    // 使用指针范围赋值
    void assign(const T *first, const T *last)
    {
        sz = last - first;
        if (sz > DefaultSize)
        {
            sz = DefaultSize; // 限制有效元素数量不超过 DefaultSize
        }
        for (std::size_t i = 0; i < sz; ++i)
        {
            data[i] = first[i];
        }
    }

    // 获取大小
    std::size_t size() const
    {
        return sz;
    }

    // 检查是否为空
    bool empty() const
    {
        return sz == 0;
    }

    // 清空向量
    void clear()
    {
        sz = 0;
    }

    // 添加元素
    void push_back(const T &value)
    {
        if (sz < DefaultSize)
        {
            data[sz++] = value; // 不超过最大容量，正常添加
        }
        else
        {
            clear();         // 超过最大容量，先清空
            data[0] = value; // 然后将新元素放在首位
            sz = 1;          // 更新元素个数
        }
    }

    // 删除元素
    void pop_back()
    {
        if (sz > 0)
        {
            --sz;
        }
    }

    // 访问元素
    T &operator[](std::size_t index)
    {
        if (index >= sz)
        {
            static T default_value = T();
            return default_value;
        }
        return data[index];
    }

    const T &operator[](std::size_t index) const
    {
        if (index >= sz)
        {
            static T default_value = T();
            return default_value;
        }
        return data[index];
    }

    // 获取内部数据指针
    T *_data()
    {
        return data;
    }

    // 获取内部数据指针（const 版本）
    const T *_data() const
    {
        return data;
    }

private:
    T data[DefaultSize]; // 固定大小的数组
    std::size_t sz = 0;      // 当前元素个数
};

// 越界则清空
template <typename T, std::size_t MaxCapacity = 15>
class myqueue
{
public:
    // 默认构造函数
    myqueue() : front(0), rear(0), sz(0) {}

    // 拷贝构造函数：深拷贝
    myqueue(const myqueue &other)
        : front(other.front), rear(other.rear), sz(other.sz)
    {
        // 深拷贝：复制数组中的元素
        for (std::size_t i = 0; i < sz; ++i)
        {
            data[i] = other.data[i];
        }
    }

    // 拷贝赋值运算符：深拷贝
    myqueue &operator=(const myqueue &other)
    {
        if (this != &other)
        { // 防止自赋值
            front = other.front;
            rear = other.rear;
            sz = other.sz;

            // 深拷贝：复制数组中的元素
            for (std::size_t i = 0; i < sz; ++i)
            {
                data[i] = other.data[i];
            }
        }
        return *this;
    }

    // 入队操作
    void push(const T &value)
    {
        if (sz == MaxCapacity)
        {
            // 队列满时不再插入新元素,而是去掉第一个元素
            pop();
        }
        data[rear] = value;
        rear = (rear + 1) % MaxCapacity;
        ++sz;
    }

    // 出队操作
    void pop()
    {
        if (empty())
        {
            return;
        }
        front = (front + 1) % MaxCapacity;
        --sz;
    }

    // 获取队列头部元素
    T &front_element()
    {
        if (empty())
        {
            static T default_value = T(); // 确保 T 有一个默认构造函数
            return default_value;
        }
        return data[front];
    }

    const T &front_element() const
    {
        if (empty())
        {
            static T default_value = T(); // 确保 T 有一个默认构造函数
            return default_value;
        }
        return data[front];
    }

    // 获取队列尾部元素
    T &back()
    {
        if (empty())
        {
            static T default_value = T(); // 确保 T 有一个默认构造函数
            return default_value;
        }
        return data[(rear - 1 + MaxCapacity) % MaxCapacity];
    }

    const T &back() const
    {
        if (empty())
        {
            static T default_value = T(); // 确保 T 有一个默认构造函数
            return default_value;
        }
        return data[(rear - 1 + MaxCapacity) % MaxCapacity];
    }

    // 检查队列是否为空
    bool empty() const
    {
        return sz == 0;
    }

    // 获取队列的大小
    std::size_t size() const
    {
        return sz;
    }

    // 清空队列
    void clear()
    {
        front = 0;
        rear = 0;
        sz = 0;
    }
		
		float getAverage()
		{
			float sum=0;
			for(u8 i=0;i<sz;i++)
			{
				sum+=data[i];
			}
			return sum/(float)sz;
		}

private:
    T data[MaxCapacity] ; // 固定大小的数组，存储队列元素
    std::size_t front = 0;   // 队列头部索引
    std::size_t rear = 0;    // 队列尾部索引
    std::size_t sz = 0;      // 当前队列大小
};

// 数据类型枚举
enum DataType {
    type_int,
		type_uint,
    type_float,
    type_bool,
    type_unknown
};

// 存储一个数据项的结构体
struct Mpack {
    const char* name;  // 变量名称
    DataType type;     // 数据类型
    void *data;        // 数据地址
    size_t size;       // 数据大小
};

// 简易MPack数据结构（用于保存数据项的集合）
class SimpleMPack 
{
	public:
  myvector<Mpack,20> Mpacks;
};

#endif

void boardALLInit(void);
void softwareReset(void);
extern u8 Init_OK;
extern u8 flashNum;


#endif /* __BOARD_H__ */

#endif /* __MASK__ */
