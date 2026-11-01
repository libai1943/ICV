# Chapter 8: upstream CasADi C++ example

The initial example was developed by Yueshuo Sun. This chapter-local copy preserves its three-variable NLP and its standalone CMake build. The adjacent `../casadi_example.cc` is the pre-existing book example; it has been preserved.

## Provenance

- Original repository: https://github.com/YueshuoSun/casadi_example
- Original branch: `main`
- Exact imported commit: `6314f25f7608dd149cbce2780fc79dc0087023ed`
- Snapshot: https://github.com/YueshuoSun/casadi_example/tree/6314f25f7608dd149cbce2780fc79dc0087023ed
- Book repository: https://github.com/libai1943/ICV

## Editorial changes

- Translated Chinese comments and user-facing messages into English.
- Consolidated Yueshuo Sun's first-party contribution headers in this README.
- Preserved mathematical expressions, identifiers, ROS interfaces, configuration values, and third-party notices, except for the specific changes listed below.

- Selected `ma57` instead of the upstream `ma27`, consistently with the book.

The numerical objective, constraint, starting point, and result extraction are unchanged. The example has not been built against a local CasADi/HSL installation in this import environment.
