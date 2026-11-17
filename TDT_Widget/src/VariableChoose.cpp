#include "VariableChoose.h"

// 构造函数：初始化菜单项
VariableChoose::VariableChoose(Widget *parent, SimpleMPack &simpleMPack)
    : Menu(parent), simpleMPack(simpleMPack) {
			itemCount=simpleMPack.Mpacks.size();
}

// 显示菜单：展示所有菜单项
void VariableChoose::show() 
{
	for (u8 i = 0; i < simpleMPack.Mpacks.size(); ++i)
	{
		//simpleMPack.dataItems[itemIndex].name;
	}
	
//    auto oldDrawRect = oled.setDrawRect();

//    oled.FillRectangle(0, 0, 128, 16, 0);  // 清空标题区域
//    oled.ShowString(0, 0, (u8 *)"Select Item:", 8, 1);  // 显示标题

//    // 显示菜单项
//    for (u8 i = 0; i < itemCount; ++i) {
//        if (i == itemIndex) {
//            oled.FillRectangle(0, 16 + i * 10, 128, 16 + (i + 1) * 10, 1);  // 高亮显示当前选中项
//        }
//        oled.ShowString(10, 16 + i * 10, (u8 *)itemList[i].name, 8, 1);
//    }

//    oled.setDrawRect(oldDrawRect);
}

// 按下 OK 键时显示数据详情
void VariableChoose::btnOK_Press() {
    displayItemData(itemIndex);
}

// 显示选中项的数据
void VariableChoose::displayItemData(u8 index) {
//    SimpleMPack::DataItem selectedItem = itemList[index].dataItem;
//    DataType type = itemList[index].type;
//    char buffer[40];

//    // 显示数据的名称
//    oled.FillRectangle(0, 0, 128, 16, 0);
//    oled.ShowString(0, 0, (u8 *)itemList[index].name, 8, 1);

//    // 根据数据类型显示对应的值
//    switch (type) {
//        case type_uint: {
//            sprintf(buffer, "Value: %u", *(uint32_t*)selectedItem.data);
//            break;
//        }
//        case type_int: {
//            sprintf(buffer, "Value: %d", *(int32_t*)selectedItem.data);
//            break;
//        }
//        case type_float: {
//            sprintf(buffer, "Value: %.2f", *(float*)selectedItem.data);
//            break;
//        }
//        case type_bool: {
//            sprintf(buffer, "Value: %s", *(bool*)selectedItem.data ? "True" : "False");
//            break;
//        }
//        default:
//            sprintf(buffer, "Unknown Type");
//            break;
//    }

//    // 显示值
//    oled.FillRectangle(0, 16, 128, 32, 0);
//    oled.ShowString(0, 16, (u8 *)buffer, 8, 1);
}
