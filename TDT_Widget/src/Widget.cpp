// Widget ���ʵ��
#include "Widget.h"
Widget *Widget::currentWidget = nullptr;
u8 Widget::keyStatus = 0;

// ����ɨ�躯��
void Widget::key_Scan() {
    // ���谴��ɨ���Ѿ�ʵ��
    // ���� keyStatus
    keyStatus = 0;  // ���°���״̬������ɨ��ʵ�ֿ��Ը���Ӳ���������
    if (keyStatus & KEY_UP) btnUp_Press();
    if (keyStatus & KEY_DOWN) btnDown_Press();
    if (keyStatus & KEY_OK) btnOK_Press();
    if (keyStatus & KEY_CANCEL) btnCancel_Press();
}

// ���� Cancel ��ť�����¼�
void Widget::btnCancel_Press()
{
    backToParent(); // ���ص������
}


Menu::Menu(Widget *parent) : Widget(parent), itemCount(0), itemIndex(0) {}

void Menu::addItem(WidgetDescription *item)
{
    if (itemCount < MAX_MENU_ITEMS)
    {
        itemList[itemCount++] = item;  // ���Ӳ˵���
    }
}

void Menu::show()
{
//    auto oldDrawRect = oled.setDrawRect(); // ���浱ǰ��������
//    if (widgetTitle != 0)  // ����˵��������
//    {
//        oled.FillRectangle(0, menuShowRowFrom * 8, 127, (menuShowRowFrom + 1) * 8 - 1, 0); // �����������
//        oled.ShowString(0, menuShowRowFrom * 8, (u8 *)widgetTitle, 8, 1);  // ��ʾ����
//    }

//    oled.DrawLine(0, (menuShowRowFrom + 1) * 8, 128, (menuShowRowFrom + 1) * 8 - 1, 1); // ���Ʒָ���
//    oled.setDrawRect(0, (menuShowRowFrom + 1) * 8, 128, (menuShowRowFrom + 1 + menuShowRowNum) * 8 - 1); // ���û�������
//    oled.FillRectangle(0, 0, 127, (menuShowRowNum) * 8 - 1, 0, CM_Fill); // ����˵���������

//    // ��ʾ�˵���
//    for (int i = 0; i < menuShowRowNum; i++)
//    {
//        if (itemIndex - (itemIndex % menuShowRowNum) + i >= itemCount) // ��������˵�������������ѭ��
//            break;

//        if (i == itemIndex % menuShowRowNum)  // �����ǰ����ѡ�еĲ˵��������ʾ
//            oled.ShowString(0, (i) * 8, (u8 *)itemList[itemIndex - (itemIndex % menuShowRowNum) + i]->descriptionStr, 8, 1, CM_Xor);
//        else  // ����������ʾ
//            oled.ShowString(0, (i) * 8, (u8 *)itemList[itemIndex - (itemIndex % menuShowRowNum) + i]->descriptionStr, 7, 1, CM_Xor);
//    }

//    oled.FillRectangle(0, ((itemIndex % menuShowRowNum)) * 8, 127, ((itemIndex % menuShowRowNum) + 1) * 8 - 1, 1, CM_Xor); // ������ʾ��ǰѡ�еĲ˵����
//    oled.setDrawRect(oldDrawRect); // �ָ�֮ǰ�Ļ�������
}

void Menu::btnUp_Press()
{
    itemIndex--;  // ����ѡ��˵���
    if (itemIndex == 0xff)
        itemIndex = itemCount - 1;
}

void Menu::btnDown_Press()
{
    itemIndex++;  // ����ѡ��˵���
    if (itemIndex >= itemCount)
        itemIndex = 0;
}

void Menu::btnOK_Press()
{
    if (itemList[itemIndex] && itemList[itemIndex]->callback)  // ����ص��������ڣ�������
    {
        itemList[itemIndex]->callback();
    }
}
