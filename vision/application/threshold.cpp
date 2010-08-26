// Copyright 2009 Erik Weitnauer
/// Demonstrates how to convert a color image to binary with a reference color and threshold.

#include <ICLQuick/Common.h>
#include <QtGui/QPushButton>

GUI gui("vbox[@handle=B]");

std::string create_camcfg(const std::string&, const std::string &hint){
  return str("camcfg(")+hint+")[@maxsize=5x2]";
}

void init(){
    gui << "image[@handle=myimage]";
    gui << create_camcfg(FROM_PROGARG("-input"));
    gui << "slider(0,255,20)[@out=t@label=threshold]";
    gui.show();
}

const Img8u &thresh(const Img8u &input, icl8u t){
  static Img8u result;
  result.setChannels(1);
  result.setSize(input.getSize());
  
  int t3 = 3*t;
  for(int x=0;x<input.getWidth();++x){
    for(int y=0;y<input.getHeight();++y){
      result(x,y,0) = 255*((input(x,y,0)+input(x,y,1)+input(x,y,2))>t3);
    }
  }
  return result;
}

void myrun(){
  static GenericGrabber grabber(FROM_PROGARG("-input"));
  grabber.setDesiredDepth(depth8u);
  
  const Img8u &image = *grabber.grab()->asImg<icl8u>();
  
  const Img8u &tImage = thresh(image,gui.getValue<int>("t"));
  
  gui.getValue<ImageHandle>("myimage") = tImage;
  gui.getValue<ImageHandle>("myimage").update();  
  Thread::msleep(10);
}


int main(int n, char **args){
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
  painit(n,args,"-input|-i(device-type=dc,device-params=0)");
	QApplication app(n,args);
  ExecThread x(myrun);
  
  init();
  
  x.run();
  return app.exec();
}
