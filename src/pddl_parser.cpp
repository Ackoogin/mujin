#include "mujin/pddl_parser.h"
#include "mujin/world_model.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace mujin {

// =========================================================================
// Tokenizer
// =========================================================================

static std::vector<std::string> tokenize(const std::string& input) {
    std::vector<std::string> tokens;
    size_t i = 0;
    while (i < input.size()) {
        // Skip whitespace
        if (std::isspace(static_cast<unsigned char>(input[i]))) {
            ++i;
            continue;
        }
        // Skip comments (;)
        if (input[i] == ';') {
            while (i < input.size() && input[i] != '\n') ++i;
            continue;
        }
        // Parentheses are individual tokens
        if (input[i] == '(' || input[i] == ')') {
            tokens.push_back(std::string(1, input[i]));
            ++i;
            continue;
        }
        // Accumulate a word token
        size_t start = i;
        while (i < input.size() && !std::isspace(static_cast<unsigned char>(input[i]))
               && input[i] != '(' && input[i] != ')' && input[i] != ';') {
            ++i;
        }
        std::string tok = input.substr(start, i - start);
        // Lowercase for case-insensitive PDDL
        std::transform(tok.begin(), tok.end(), tok.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        tokens.push_back(tok);
    }
    return tokens;
}

// =========================================================================
// S-expression tree
// =========================================================================

struct SExpr {
    bool is_atom = true;
    std::string atom;
    std::vector<SExpr> children;
};

static SExpr parseSExpr(const std::vector<std::string>& tokens, size_t& pos) {
    if (pos >= tokens.size()) {
        throw std::runtime_error("PDDL parse error: unexpected end of input");
    }
    if (tokens[pos] == "(") {
        ++pos; // consume '('
        SExpr expr;
        expr.is_atom = false;
        while (pos < tokens.size() && tokens[pos] != ")") {
            expr.children.push_back(parseSExpr(tokens, pos));
        }
        if (pos >= tokens.size()) {
            throw std::runtime_error("PDDL parse error: unmatched '('");
        }
        ++pos; // consume ')'
        return expr;
    } else if (tokens[pos] == ")") {
        throw std::runtime_error("PDDL parse error: unexpected ')'");
    } else {
        SExpr expr;
        expr.is_atom = true;
        expr.atom = tokens[pos];
        ++pos;
        return expr;
    }
}

static SExpr parseAll(const std::string& input) {
    auto tokens = tokenize(input);
    size_t pos = 0;
    if (tokens.empty()) {
        throw std::runtime_error("PDDL parse error: empty input");
    }
    return parseSExpr(tokens, pos);
}

// =========================================================================
// Helpers to navigate S-expressions
// =========================================================================

// Find a child whose first element is an atom matching `keyword`
static const SExpr* findSection(const SExpr& parent, const std::string& keyword) {
    for (auto& child : parent.children) {
        if (!child.is_atom && !child.children.empty() &&
            child.children[0].is_atom && child.children[0].atom == keyword) {
            return &child;
        }
    }
    return nullptr;
}

// Parse a typed list: "a b c - type d e - type2" into pairs (name, type)
// Also handles untyped lists (everything gets type "object")
static std::vector<std::pair<std::string, std::string>> parseTypedList(
    const std::vector<SExpr>& elements, size_t start)
{
    std::vector<std::pair<std::string, std::string>> result;
    std::vector<std::string> pending;

    for (size_t i = start; i < elements.size(); ++i) {
        if (!elements[i].is_atom) continue;
        const std::string& tok = elements[i].atom;
        if (tok == "-") {
            // Next token is the type
            ++i;
            if (i >= elements.size() || !elements[i].is_atom) {
                throw std::runtime_error("PDDL: expected type after '-'");
            }
            std::string type = elements[i].atom;
            for (auto& name : pending) {
                result.push_back({name, type});
            }
            pending.clear();
        } else {
            pending.push_back(tok);
        }
    }
    // Remaining items without explicit type get "object"
    for (auto& name : pending) {
        result.push_back({name, "object"});
    }
    return result;
}

// Parse a predicate atom like (at ?r ?loc) from an S-expression
// Returns: predicate name, list of argument names
static std::pair<std::string, std::vector<std::string>> parseAtom(const SExpr& expr) {
    if (expr.is_atom) {
        // 0-ary predicate
        return {expr.atom, {}};
    }
    if (expr.children.empty() || !expr.children[0].is_atom) {
        throw std::runtime_error("PDDL: malformed atom");
    }
    std::string name = expr.children[0].atom;
    std::vector<std::string> args;
    for (size_t i = 1; i < expr.children.size(); ++i) {
        if (expr.children[i].is_atom) {
            args.push_back(expr.children[i].atom);
        }
    }
    return {name, args};
}

// =========================================================================
// Domain parsing
// =========================================================================

struct PddlAction {
    std::string name;
    std::vector<std::string> param_names;
    std::vector<std::string> param_types;
    std::vector<std::string> precondition_atoms;  // fluent key templates
    std::vector<std::string> add_atoms;
    std::vector<std::string> del_atoms;
};

// Build a fluent key template from predicate name + args
// e.g. ("at", {"?r", "?from"}) -> "(at ?r ?from)"
static std::string buildFluentTemplate(const std::string& pred,
                                       const std::vector<std::string>& args) {
    std::string key = "(" + pred;
    for (auto& a : args) {
        key += " " + a;
    }
    key += ")";
    return key;
}

// Parse precondition (handles bare atom and (and ...))
static void parsePrecondition(const SExpr& expr,
                              std::vector<std::string>& atoms) {
    if (expr.is_atom) return;
    if (expr.children.empty()) return;

    if (expr.children[0].is_atom && expr.children[0].atom == "and") {
        for (size_t i = 1; i < expr.children.size(); ++i) {
            parsePrecondition(expr.children[i], atoms);
        }
    } else {
        auto [pred, args] = parseAtom(expr);
        atoms.push_back(buildFluentTemplate(pred, args));
    }
}

// Parse effect (handles bare atom, (and ...), (not ...))
static void parseEffect(const SExpr& expr,
                        std::vector<std::string>& add_atoms,
                        std::vector<std::string>& del_atoms) {
    if (expr.is_atom) return;
    if (expr.children.empty()) return;

    if (expr.children[0].is_atom && expr.children[0].atom == "and") {
        for (size_t i = 1; i < expr.children.size(); ++i) {
            parseEffect(expr.children[i], add_atoms, del_atoms);
        }
    } else if (expr.children[0].is_atom && expr.children[0].atom == "not") {
        if (expr.children.size() >= 2) {
            auto [pred, args] = parseAtom(expr.children[1]);
            del_atoms.push_back(buildFluentTemplate(pred, args));
        }
    } else {
        auto [pred, args] = parseAtom(expr);
        add_atoms.push_back(buildFluentTemplate(pred, args));
    }
}

static PddlAction parseAction(const SExpr& expr) {
    PddlAction act;

    // (:action name :parameters (...) :precondition (...) :effect (...))
    size_t i = 1; // skip ":action"
    if (i < expr.children.size() && expr.children[i].is_atom) {
        act.name = expr.children[i].atom;
        ++i;
    }

    while (i < expr.children.size()) {
        if (expr.children[i].is_atom) {
            const std::string& kw = expr.children[i].atom;
            ++i;
            if (i >= expr.children.size()) break;

            if (kw == ":parameters") {
                // Next is a list of typed params
                auto& params_expr = expr.children[i];
                if (!params_expr.is_atom) {
                    auto typed = parseTypedList(params_expr.children, 0);
                    for (auto& [name, type] : typed) {
                        act.param_names.push_back(name);
                        act.param_types.push_back(type);
                    }
                }
                ++i;
            } else if (kw == ":precondition") {
                parsePrecondition(expr.children[i], act.precondition_atoms);
                ++i;
            } else if (kw == ":effect") {
                parseEffect(expr.children[i], act.add_atoms, act.del_atoms);
                ++i;
            } else {
                ++i; // skip unknown keyword value
            }
        } else {
            ++i;
        }
    }

    return act;
}

// =========================================================================
// Public API
// =========================================================================

static std::string readFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("PddlParser: cannot open file '" + path + "'");
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

void PddlParser::parse(const std::string& domain_path,
                        const std::string& problem_path,
                        WorldModel& wm) {
    std::string domain_str = readFile(domain_path);
    std::string problem_str = readFile(problem_path);
    parseFromString(domain_str, problem_str, wm);
}

void PddlParser::parseFromString(const std::string& domain_pddl,
                                  const std::string& problem_pddl,
                                  WorldModel& wm) {
    auto domain = parseAll(domain_pddl);
    auto problem = parseAll(problem_pddl);

    // ===================== Parse domain =====================

    // Types
    auto* types_section = findSection(domain, ":types");
    if (types_section) {
        auto typed = parseTypedList(types_section->children, 1);
        // Ensure "object" base type exists
        wm.typeSystem().addType("object");
        for (auto& [name, parent] : typed) {
            if (!wm.typeSystem().hasType(parent) && parent != "object") {
                wm.typeSystem().addType(parent);
            }
            wm.typeSystem().addType(name, parent);
        }
    } else {
        // Untyped domain
        wm.typeSystem().addType("object");
    }

    // Predicates
    auto* pred_section = findSection(domain, ":predicates");
    if (pred_section) {
        for (size_t i = 1; i < pred_section->children.size(); ++i) {
            auto& pred_expr = pred_section->children[i];
            if (pred_expr.is_atom) {
                // 0-ary predicate
                wm.registerPredicate(pred_expr.atom, {});
            } else if (!pred_expr.children.empty()) {
                std::string pred_name = pred_expr.children[0].atom;
                auto typed = parseTypedList(pred_expr.children, 1);
                std::vector<std::string> param_types;
                for (auto& [name, type] : typed) {
                    param_types.push_back(type);
                }
                wm.registerPredicate(pred_name, param_types);
            }
        }
    }

    // Actions
    std::vector<PddlAction> actions;
    for (auto& child : domain.children) {
        if (!child.is_atom && !child.children.empty() &&
            child.children[0].is_atom && child.children[0].atom == ":action") {
            actions.push_back(parseAction(child));
        }
    }

    // ===================== Parse problem =====================

    // Objects
    auto* obj_section = findSection(problem, ":objects");
    if (obj_section) {
        auto typed = parseTypedList(obj_section->children, 1);
        for (auto& [name, type] : typed) {
            wm.addObject(name, type);
        }
    }

    // Now register actions (after objects so grounding works)
    for (auto& act : actions) {
        wm.registerAction(act.name, act.param_names, act.param_types,
                          act.precondition_atoms, act.add_atoms, act.del_atoms);
    }

    // Init
    auto* init_section = findSection(problem, ":init");
    if (init_section) {
        for (size_t i = 1; i < init_section->children.size(); ++i) {
            auto [pred, args] = parseAtom(init_section->children[i]);
            std::string key = buildFluentTemplate(pred, args);
            wm.setFact(key, true);
        }
    }

    // Goal
    auto* goal_section = findSection(problem, ":goal");
    if (goal_section && goal_section->children.size() >= 2) {
        std::vector<std::string> goal_atoms;
        parsePrecondition(goal_section->children[1], goal_atoms);
        wm.setGoal(goal_atoms);
    }
}

} // namespace mujin
