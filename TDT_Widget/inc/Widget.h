#ifndef __WIDGET_H
#define __WIDGET_H

#include "board.h"
#define MAX_MENU_ITEMS 20
// 按键状态
enum KeyStatus {
    KEY_UP = 0x01,
    KEY_DOWN = 0x02,
    KEY_OK = 0x03,
    KEY_CANCEL = 0x04
};

// Widget 类，定义 UI 组件
class Widget
{
public:
    Widget(Widget *parent = nullptr) : parent(parent), widgetTitle(nullptr), notFirstDraw(0){}

    // 显示组件标题
    void showTitle();
		
		//设置组件标题	
		void setTitle(char *widgetTitle){this->widgetTitle = widgetTitle;}

    // 返回上一级组件
    void backToParent() {
        if (parent) currentWidget = parent;
    }

    static void setCurrentWidget(Widget *widget) { currentWidget = widget; }
		
		static u8 keyStatus;
		// 按键扫描函数
    void key_Scan();
    // 按键事件处理函数
    virtual void btnUp_Press() {}
    virtual void btnDown_Press() {}
    virtual void btnOK_Press() {}
    virtual void btnCancel_Press();

    // 组件的显示函数
    virtual void show() {}

protected:
    Widget *parent;          // 父组件
    static Widget *currentWidget; // 当前组件
    char *widgetTitle;  // 组件的标题
    u8 notFirstDraw;    // 是否是第一次绘制
};


// 组件描述结构体，包含回调函数指针
struct WidgetDescription
{
    char *descriptionStr;  // 描述字符串
    void (*callback)();    // 回调函数指针
};

// Menu 类，继承自 Widget，实现菜单功能
class Menu : public Widget
{
public:
    WidgetDescription *itemList[MAX_MENU_ITEMS]; // 固定大小数组
    u8 itemCount = 0;  //总菜单数
    u8 itemIndex = 0;  //总组件数

		Menu(Widget *parent);

    void addItem(WidgetDescription *item);    // 添加菜单项

    virtual void show() override; // 显示菜单
    virtual void btnUp_Press() override; // 向上切换菜单项
    virtual void btnDown_Press() override; // 向下切换菜单项
    virtual void btnOK_Press() override;  // 按下 OK 键时调用回调

    u8 menuShowRowNum = 0;  //屏幕最多显示的菜单数
    u8 menuShowRowFrom = 0;
};
#endif


