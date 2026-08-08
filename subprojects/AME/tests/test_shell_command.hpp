/// Helpers for the few tests that run one of AME's command-line binaries
/// through std::system(), which behaves differently on Windows and on Linux.
///
/// Two differences matter:
///
///  * The device that throws output away is named "/dev/null" on Linux and
///    "NUL" on Windows. Writing "/dev/null" on Windows does not throw the
///    output away. It creates a real file called "null" in a directory called
///    "dev" on the current drive, and two tests running at once then fail
///    because they are both writing to it.
///
///  * On Windows std::system() runs the command through "cmd /c". When the
///    command starts with a double quote, cmd removes that quote and the last
///    quote in the line before running anything, which splits the program name
///    in half. Wrapping the whole command in one more pair of quotes gives cmd
///    the pair it expects to remove and leaves the command itself intact.
#ifndef AME_TESTS_SHELL_COMMAND_HPP
#define AME_TESTS_SHELL_COMMAND_HPP

#include <filesystem>
#include <string>

namespace ame_test {

/// Quotes a path so a shell treats it as one argument even with spaces in it.
inline std::string shellQuote(const std::filesystem::path& path) {
  return "\"" + path.string() + "\"";
}

/// The redirection that discards a command's output on this platform.
inline std::string discardOutput() {
#if defined(_WIN32)
  return " > NUL 2>&1";
#else
  return " > /dev/null 2>&1";
#endif
}

/// Prepares a command for std::system(). Pass the command with its arguments
/// already quoted; the return value is what std::system() should be given.
inline std::string shellCommand(const std::string& command) {
#if defined(_WIN32)
  return "\"" + command + "\"";
#else
  return command;
#endif
}

}  // namespace ame_test

#endif  // AME_TESTS_SHELL_COMMAND_HPP
