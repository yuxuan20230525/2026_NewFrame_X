// Widget 类的实现
#include "widget.h"
Widget *Widget::currentWidget = nullptr;
u8 Widget::keyStatus = 0;

// 按键扫描函数
void Widget::key_Scan() {
    // 假设按键扫描已经实现
    // 更新 keyStatus
    keyStatus = 0;  // 更新按键状态，按键扫描实现可以根据硬件情况调整
    if (keyStatus & KEY_UP) btnUp_Press();
    if (keyStatus & KEY_DOWN) btnDown_Press();
    if (keyStatus & KEY_OK) btnOK_Press();
    if (keyStatus & KEY_CANCEL) btnCancel_Press();
}

// 处理 Cancel 按钮按下事件
void Widget::btnCancel_Press()
{
    backToParent(); // 返回到父组件
}


Menu::Menu(Widget *parent) : Widget(parent), itemCount(0), itemIndex(0) {}

void Menu::addItem(WidgetDescription *item)
{
    if (itemCount < MAX_MENU_ITEMS)
    {
        itemList[itemCount++] = item;  // 添加菜单项
    }
}

void Menu::show()
{
//    auto oldDrawRect = oled.setDrawRect(); // 保存当前绘制区域
//    if (widgetTitle != 0)  // 如果菜单标题存在
//    {
//        oled.FillRectangle(0, menuShowRowFrom * 8, 127, (menuShowRowFrom + 1) * 8 - 1, 0); // 清除标题区域
//        oled.ShowString(0, menuShowRowFrom * 8, (u8 *)widgetTitle, 8, 1);  // 显示标题
//    }

//    oled.DrawLine(0, (menuShowRowFrom + 1) * 8, 128, (menuShowRowFrom + 1) * 8 - 1, 1); // 绘制分隔线
//    oled.setDrawRect(0, (menuShowRowFrom + 1) * 8, 128, (menuShowRowFrom + 1 + menuShowRowNum) * 8 - 1); // 设置绘制区域
//    oled.FillRectangle(0, 0, 127, (menuShowRowNum) * 8 - 1, 0, CM_Fill); // 清除菜单内容区域

//    // 显示菜单项
//    for (int i = 0; i < menuShowRowNum; i++)
//    {
//        if (itemIndex - (itemIndex % menuShowRowNum) + i >= itemCount) // 如果超出菜单项数量，跳出循环
//            break;

//        if (i == itemIndex % menuShowRowNum)  // 如果当前项是选中的菜单项，高亮显示
//            oled.ShowString(0, (i) * 8, (u8 *)itemList[itemIndex - (itemIndex % menuShowRowNum) + i]->descriptionStr, 8, 1, CM_Xor);
//        else  // 否则正常显示
//            oled.ShowString(0, (i) * 8, (u8 *)itemList[itemIndex - (itemIndex % menuShowRowNum) + i]->descriptionStr, 7, 1, CM_Xor);
//    }

//    oled.FillRectangle(0, ((itemIndex % menuShowRowNum)) * 8, 127, ((itemIndex % menuShowRowNum) + 1) * 8 - 1, 1, CM_Xor); // 高亮显示当前选中的菜单项背景
//    oled.setDrawRect(oldDrawRect); // 恢复之前的绘制区域
}

void Menu::btnUp_Press()
{
    itemIndex--;  // 向上选择菜单项
    if (itemIndex == 0xff)
        itemIndex = itemCount - 1;
}

void Menu::btnDown_Press()
{
    itemIndex++;  // 向下选择菜单项
    if (itemIndex >= itemCount)
        itemIndex = 0;
}

void Menu::btnOK_Press()
{
    if (itemList[itemIndex] && itemList[itemIndex]->callback)  // 如果回调函数存在，调用它
    {
        itemList[itemIndex]->callback();
    }
}
