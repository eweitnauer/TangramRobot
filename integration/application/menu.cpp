// Copyright 2009 Erik Weitnauer
/// Test-GUI with some buttons.

#include "ICLQuick/Common.h"
#include <iostream>
#include <QtGui/QTextEdit>

GUI gui("vsplit");
LabelHandle m_label;

void btn1_clicked() {
	cout << "Button 1 clicked." << endl;	
}

void btn2_clicked() {
	cout << "Button 2 clicked." << endl;	
}

void btn3_clicked() {
	cout << "Button 3 clicked." << endl;	
}

void btn4_clicked() {
	cout << "Button 4 clicked." << endl;	
}

void main_loop() {
	Thread::msleep(10);
}

void init() {
	GUI hbox1("hbox[@minsize=20x2]");
	hbox1 << "button(Button 1)[@handle=btn1]"
				<< "button(Button 2)[@handle=btn2]"
				<< "button(Button 3)[@handle=btn3]"
				<< "button(Button 4)[@handle=btn4]";
	GUI hbox2("hbox[@handle=edit@minsize=20x10]");
	gui << hbox1 << hbox2;
	
	gui.show();
	
	GUI g("button(Hi)");
	g.show();

	gui.getValue<ButtonHandle>("btn1").registerCallback(new GUI::Callback(btn1_clicked));
	gui.getValue<ButtonHandle>("btn2").registerCallback(new GUI::Callback(btn2_clicked));
	gui.getValue<ButtonHandle>("btn3").registerCallback(new GUI::Callback(btn3_clicked));
	gui.getValue<ButtonHandle>("btn4").registerCallback(new GUI::Callback(btn4_clicked));
	QTextEdit *edit = new QTextEdit();
	edit->insertPlainText("Hallo\nWelt!");
	gui.getValue<BoxHandle>("edit").add(edit);
}

int main(int n, char **args) {
	ICLApplication app(n, args, "", init, main_loop);
	return app.exec();
}
