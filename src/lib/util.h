#include <gtsam/geometry/Point3.h>
#include <math.h>

using namespace gtsam;
using namespace std;

double norm(Point3 vec);

map<string,Vector> dataToVecMap(json jsonObj);