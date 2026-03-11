# Core Module - C++ Coding Style Guide

This document summarizes the key coding style and C++ usage conventions followed.

## General Principles

*   **Modern C++:** Utilize features from C++11/14 where appropriate (`auto`, range-based for loops, smart pointers, etc.). Follow official ISO C++ standards and guidelines.
*   **STL:** Leverage the Standard Template Library (STL) and standard algorithms (`<algorithm>`, `<vector>`, `<string>`, etc.) for common tasks.
*   **Readability:** Write concise, idiomatic, and clear C++ code. Use descriptive names for variables, functions, and classes.
*   **Namespaces:** Organize code logically using namespaces.

## Formatting and Naming

*   **Indentation:** Use 2 spaces for indentation.
*   **Braces:** Place opening braces (`{`) on the same line as the control structure (if, for, while) or function/method declaration.
*   **Comments:**
    *   Use `/// \brief` format for documenting public APIs (classes, methods, enums, etc.) for Doxygen and VS Code compatibility.
    *   Example: `/// \brief Calculates the distance between two points`
    *   Do not use C-style block comments (`/* */`) or other documentation formats (`/** */`, `/// <summary>`, etc.).
    *   Avoid obvious comments that merely restate the code.
    *   Do not write change history or author information into comments.
*   **Naming Conventions:**
    *   **Classes/Structs:** `CapitalCase`
    *   **Methods/Functions:** `camelCase`
    *   **Variables/Members:** `snake_case` (including private members, e.g., `my_variable_`)

## Memory and Resource Management

*   **RAII:** Employ the Resource Acquisition Is Initialization (RAII) principle consistently (e.g., using `std::lock_guard`, `std::unique_lock`, smart pointers).
*   **Smart Pointers:**
    *   Prefer `std::unique_ptr` for exclusive ownership.
    *   Use `std::shared_ptr` for shared ownership scenarios.
    *   Avoid raw owning pointers (`new`/`delete`). Use smart pointers for memory management.
*   **Allocation:** Avoid unnecessary heap allocations. Prefer stack allocation where feasible.
*   **Move Semantics:** Use `std::move` to enable move semantics and avoid unnecessary copies where applicable.
*   **Containers:** Prefer `std::array` or `std::vector` over raw C-style arrays.

## C++ Features and Type Safety

*   **Enums:** Use `enum class` for strongly-typed enumerations.
*   **Constants:** Use `const` extensively for variables, parameters, and member functions that do not modify state. Use `constexpr` for compile-time constants where possible.
*   **Casting:** Avoid C-style casts. Use C++ casts (`static_cast`, `dynamic_cast`, `reinterpret_cast`, `const_cast`) appropriately and only when necessary.
*   **Const Correctness:** Enforce `const`-correctness strictly in function signatures and member variables.

## Error Handling

*   **Exceptions:** Use exceptions for reporting and handling exceptional runtime errors (e.g., `std::runtime_error`, `std::invalid_argument`). Use error codes or `std::pair<T, bool>` / `std::optional<T>` for non-exceptional failures (e.g., parsing failures, search misses).
*   **Input Validation:** Validate function inputs at boundaries where necessary.

## Code Structure

*   **Separation:** Separate interface (`.h`) from implementation (`.cpp`).
*   **Globals:** Avoid global variables.
*   **Singletons:** Use singletons sparingly and only when clearly justified.
*   **Templates:** Use templates and metaprogramming judiciously for generic programming solutions.

## Testing

*   **Unit Tests:** Write unit tests for components to verify functionality.
*   **Integration Tests:** Implement integration tests for interactions between components where applicable.

## Documentation

*   **API:** Document all public interfaces (classes, methods, etc.) using `/// \brief` comments.
*   **Logic:** Add comments to explain complex algorithms, non-trivial logic, assumptions, or constraints. 