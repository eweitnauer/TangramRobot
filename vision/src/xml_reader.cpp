#include <xml_reader.h>
#include <ICLQuick/Common.h>
#include <ICLUtils/XMLDocument.h>
#include <ICLUtils/ConfigFile.h>

void XMLReader::readShapes(const std::string &filename,std::vector<PolygonShape> &shapes, 
			                      float &base_length, float &height){
	ConfigFile f(filename);
	base_length = f["config.base_length"];
  height = f["config.height"];
	for(int i=1;true;i++){
		f.setPrefix("config.shape-"+str(i)+".");
	  try{
		  std::string name = f["name"];
		  std::vector<float> corners = parseVecStr<float>(f["corners"]);
		  float corner_scaling = f["corner_scaling"];
		  shapes.push_back(PolygonShape(name, height));
		  for (unsigned int i=0; i<corners.size(); i+=2) {
				 shapes.back().addCorner(Point32f(corners[i], corners[i+1])*corner_scaling*base_length);
			}
			shapes.back().updateGeometry();
		} catch(...){
			break;	
	  }
	}
}
