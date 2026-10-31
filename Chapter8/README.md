# Chapter 8: Numerical-Stack Installation Check

`casadi_example.cc` is the small standalone C++ program printed in the illustrated installation chapter. It explicitly selects IPOPT with MA57. The analytic global solution is `(x,y,z)=(0,1,0)` with zero cost and zero equality residual.

The chapter's documented Linux baseline is Ubuntu 20.04.6, ROS Noetic, ThirdParty-HSL 2.2.5, IPOPT 3.14.17, and CasADi 3.7.0. This baseline reproduces the supplied ROS 1 application; its platform-support status and a complete installation walkthrough are explained in the book. The MATLAB experiments use the separately recorded Windows/CasADi environment.

After installing compatible headers and shared libraries:

```bash
g++ -std=c++11 casadi_example.cc -I/usr/local/include \
  -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lcasadi \
  -o casadi_example
./casadi_example
```

Check the startup banner for MA57, the termination status, the solution, and the residual. If runtime HSL loading requires a nondefault name, set `ipopt.hsllib` to the actual installed library path. Do not substitute another linear solver. HSL binaries are not included.

The installation screenshots were supplied by the student contributors and integrated into the manuscript. No Ubuntu installation, virtual machine, or ROS deployment was performed on the author's computer during the book revision. The analytic test is independently checkable; this repository does not claim a newly executed Linux installation.
