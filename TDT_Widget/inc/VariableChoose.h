#ifndef __VARIABLE_CHOOSE_H
#define __VARIABLE_CHOOSE_H

#include "Widget.h"

class VariableChoose : public Menu {
public:
    VariableChoose(Widget *parent, SimpleMPack &simpleMPack);
    virtual void show() override;  // ��ʾ�˵�
    virtual void btnOK_Press() override;  // ���� OK ��ʱ��ʾ��������

protected:
    SimpleMPack &simpleMPack;  // ���� SimpleMPack ����
  
    void displayItemData(u8 index);  // ��ʾѡ�еĲ˵�������
};

#endif // __VARIABLE_CHOOSE_H
