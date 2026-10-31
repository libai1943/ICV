function setup_solver()
location=getenv('CASADI_MATLAB_PATH');
if ~isempty(location),addpath(location);end
assert(isfile(getenv('IPOPT_HSL_LIBRARY')),'Set IPOPT_HSL_LIBRARY to a licensed, compatible HSL library containing MA57.');
assert(exist('casadi.SX','class')==8,'Set CASADI_MATLAB_PATH to a compatible CasADi MATLAB package.');
end
