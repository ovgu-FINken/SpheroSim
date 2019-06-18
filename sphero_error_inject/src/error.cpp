#include <sphero_error_inject/error.h>

namespace speheroSim {
	Error::Error(float lin, float ang): linearError {lin}, angularError{ang};
}