#ifndef CUSTOMTREEVIEW_H
#define CUSTOMTREEVIEW_H
#include <QTreeView>
#include <QWheelEvent>

class CustomTreeView : public QTreeView {
    Q_OBJECT

public:
    CustomTreeView(QWidget *parent = nullptr);

protected:
    void wheelEvent(QWheelEvent *event) override {
        if (event->modifiers() & Qt::ControlModifier) {
            // 根据滚轮滚动方向调整字体大小
            if (event->angleDelta().y() > 0) {
                // 向上滚动，减小字体大小
                fontSize -= 1;
            }
            else
            {
                // 向下滚动，增大字体大小
                fontSize += 1;
            }
            // 限制字体大小的范围，防止过大或过小
            fontSize = qBound(6, fontSize, 20);
            // 设置样式表来调整字体大小
            setStyleSheet("QTreeView { font-size: " + QString::number(fontSize) + "pt; color: white; }");
        }
        else
        {
            // 如果没有按下 Ctrl 键，则调用基类的滚轮事件处理函数
            QTreeView::wheelEvent(event);
        }

}

private:
    int fontSize;
};
#endif // CUSTOMTREEVIEW_H
