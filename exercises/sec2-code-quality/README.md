# Internal Code Quality

The term *internal code quality* is quite vague, as the quality of a piece of code can be determined by many factors.
In general, though, it refers to writing code in such a manner that it is easy to read, understand, maintain and reuse.
It is easier to read linear and modular code, than it is to read [*spaghetti code*](https://en.wikipedia.org/wiki/Spaghetti_code), for example, especially when working with a team.
It is also easier to test a small function with few `if` statements, than it is to test a large function with many possible cases.

Internal code quality is commonly assessed based on a number of metrics, such as lines of code and cyclomatic complexity, or by checking compliance with coding standards, such as the [ROS C++ Style Guide](https://wiki.ros.org/CppStyleGuide).

We tackle code quality first, in this tutorial, as it is mostly a ROS-agnostic and application-independent problem.
While refactoring the code is laborious, the analysis is fast, and the results immediate.
This step unveils general programming mistakes and bad practices that, once fixed, can potentially reduce the time spent on debugging the application-specific issues detected with other analyses, as the code becomes more maintainable.
