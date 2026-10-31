function setup_solver()
location=getenv('CASADI_MATLAB_PATH');
if ~isempty(location),addpath(location);end
assert(exist('casadi.SX','class')==8,'Set CASADI_MATLAB_PATH to a compatible CasADi MATLAB package.');
end
