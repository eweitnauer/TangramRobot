#include <PushMovement.h>

using namespace std;

std::ostream& operator<<(std::ostream &out, const PushMovement &x) {
	out << "PushMovement(from: " << x.start.getX() << ", " << x.start.getY() << ", "
	    << x.start.getZ() << " to: "  << x.end.getX() << ", " << x.end.getY() << ", "
	    << x.end.getZ() << ")";
	return out;
}
