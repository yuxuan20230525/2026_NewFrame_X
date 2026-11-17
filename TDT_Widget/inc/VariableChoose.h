#ifndef __VARIABLE_CHOOSE_H
#define __VARIABLE_CHOOSE_H

#include "widget.h"

class VariableChoose : public Menu {
public:
    VariableChoose(Widget *parent, SimpleMPack &simpleMPack);
    virtual void show() override;  // 显示菜单
    virtual void btnOK_Press() override;  // 按下 OK 键时显示数据详情

protected:
    SimpleMPack &simpleMPack;  // 引用 SimpleMPack 数据
  
    void displayItemData(u8 index);  // 显示选中的菜单项数据
};

#endif // __VARIABLE_CHOOSE_H
