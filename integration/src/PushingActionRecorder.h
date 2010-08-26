#ifndef __PUSHING_ACTION_RECORDER_H__
#define __PUSHING_ACTION_RECORDER_H__

#include <polygon_shape.h>
#include <transformation.h>
#include <iostream>
#include <fstream>

/// Class for recording, writing and reading pushing actions.
class PushingActionRecorder {
	public:
	PolygonShape shape;
	Transformation pos_begin;
	Transformation pos_end;
	icl::Point32f push_begin;
	icl::Point32f push_end;
	
	void writeToStream(std::ostream &shape_stream, std::ostream &data_stream) const {
		shape_stream << shape << std::endl;
		pos_begin.write_plain(data_stream) << std::endl;
		pos_end.write_plain(data_stream) << std::endl;
		data_stream << push_begin.x << " " << push_begin.y << std::endl;
		data_stream << push_end.x << " " << push_end.y << std::endl;
	}
};

#endif /* __PUSHING_ACTION_RECORDER_H__ */

