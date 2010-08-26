#ifndef __XML_READER_EWEITNAU_H__
#define __XML_READER_EWEITNAU_H__

#include <ICLUtils/XMLDocument.h>
#include <polygon_shape.h>

class XMLReader {
	public:
		static void readShapes(const std::string &filename,std::vector<PolygonShape> &shapes, 
			                     float &base_length, float &height);
};

#endif /* __XML_READER_EWEITNAU_H__ */

