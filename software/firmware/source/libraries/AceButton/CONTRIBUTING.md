# How to Contribute

Thank you for your interest in this library. I do appreciate any constructive
comments or suggestions about this library. If you would like to contribute
code, please do the following for non-trivial changes:

1. Create an [issue](https://github.com/bxparks/AceButton/issues) or send me
   an email so that I have some advance warning on what you would like to
   change.
1. Please rebase your branch off the 'develop' branch, and preferably squash all
   your changes into a single commit so that it's easier to review.

## Coding Style

I use the following style for this library. New code should follow the same
style for consistency and ease of diffing.

* formatting
  * 80 column lines
    * _rationale_: I often use vertically split, side-by-side editing on my
      small laptop screen.
  * 2 space indents, no tabs
  * 4 space indents for continuation lines
  * no trailing white spaces
* spacing
  * consistent and generous spaces around operators and symbols
    * e.g. `for (int i = 0; i < 10; i++) {`
    * e.g. `a = (flag) ? 3 : -1;`
    * _rationale_: Helps readability.
  * space after language keywords: e.g. `for`, `while`, `if`, etc
  * no space after function names
* pointer declaration `*` attached to the class, not the variable
  * e.g. `AceButton* button`, not `AceButton *button`
  * _rationale_: I know the latter could be argued to be technically more
    correct under the C/C++ syntax, but I think the former is more intuitive for
    many people. I've personally gone back and forth, and I decided to just pick
    a style.
* only one variable declaration per line
  * e.g. `int i, j;` not allowed, use 2 lines
  * _rationale_: Helps readability, and avoids the confusion of
   `AceButton* b1, *b2;` caused by the previous rule.
* open brace on the same line as the function name (Java style)
* naming conventions
  * class names: CamelCase
    * e.g. `MyClass`, `YourClass`
  * methods: camelCase
    * e.g. `doSomething()`, `isCondition()`, etc
    * _rationale_: Seems like the Arduino convention. Helps readability.
  * class static constants: 'k' followed by CamelCase
    * e.g. `kSomeConstant`
    * _rationale_: Prevents conflicts with `#define` macros which use the
      `ALL_CAPS_MACRO` pattern. Since AceButton is a library, I cannot predict
      which other libraries may be used by the end-user. If there is a macro
      conflict, I have no way to fix the problem.
    * in user-land codes, `ALL_CAPS` for constants would be ok because if
      there's a conflict, you can change it
  * member variables: 'm' followed by CamelCase
    * e.g. `mSomeVariable`
    * _rationale_: Many symbols beginning with a single or double underscore
      `__` are reserved by the C language, C++ language, or their standard
      libraries. So I avoid them completely. One alternative is to append an
      underscore *after* the variable name. But this makes the `->` and the `.`
      operators hard to read. The 'm' prefix seems consistent with the 'k'
      prefix for constants, and it's easy on the eyes.
  * global variables
    * there ought to be no global variables in this library
    * if there were any, the naming convention would be 'gCamelCase'
* [doxygen](http://www.doxygen.org) comments for all public and protected
  methods and constants
  * comments are recommended for private methods and variables as well, since
    private methods sometimes become protected or public

## Unit Tests

Any non-trivial change should have a unit test. Even a seemingly
trivial change can often use a unit test to prevent typos.

Make sure that all the unit tests under the `tests` directory pass. I had to
split the unit tests into multiple `*.ino` files because they became too big to
fit into the 32KB flash memory space of an Arduino board.

## Authorship and License

I will assume that your code is licensed under the same MIT License as
the rest of the library.
