#include "ame/pddl_parser.h"
#include "ame/world_model.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ame {

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
    SExpr expr = parseSExpr(tokens, pos);
    if (pos != tokens.size()) {
        throw std::runtime_error("PDDL parse error: unexpected token after top-level expression");
    }
    return expr;
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
// Encode an `(either a b ...)` type expression as the union token "a|b|...".
static std::string parseEitherType(const SExpr& expr, const std::string& context) {
    // expr.children[0].atom == "either"
    if (expr.children.size() < 2) {
        throw std::runtime_error("PDDL: malformed (either ...) in " + context +
                                 ": expected at least one type");
    }
    std::string token;
    for (size_t j = 1; j < expr.children.size(); ++j) {
        if (!expr.children[j].is_atom) {
            throw std::runtime_error("PDDL: malformed (either ...) in " + context +
                                     ": type names must be atoms");
        }
        if (!token.empty()) token += "|";
        token += expr.children[j].atom;
    }
    return token;
}

static std::vector<std::pair<std::string, std::string>> parseTypedList(
    const std::vector<SExpr>& elements,
    size_t start,
    const std::string& context = "typed list",
    bool allow_either = false)
{
    std::vector<std::pair<std::string, std::string>> result;
    std::vector<std::string> pending;

    for (size_t i = start; i < elements.size(); ++i) {
        const SExpr& elem = elements[i];
        if (elem.is_atom && elem.atom == "-") {
            if (pending.empty()) {
                throw std::runtime_error("PDDL: malformed " + context + ": expected name before '-'");
            }
            // Next token is the type: a bare atom, or an (either ...) union
            // when permitted (action parameters only).
            ++i;
            if (i >= elements.size()) {
                throw std::runtime_error("PDDL: malformed " + context + ": expected type after '-'");
            }
            std::string type;
            if (elements[i].is_atom) {
                type = elements[i].atom;
            } else if (allow_either && !elements[i].children.empty() &&
                       elements[i].children[0].is_atom &&
                       elements[i].children[0].atom == "either") {
                type = parseEitherType(elements[i], context);
            } else {
                throw std::runtime_error("PDDL: malformed " + context + ": expected type after '-'");
            }
            for (auto& name : pending) {
                result.push_back({name, type});
            }
            pending.clear();
        } else if (elem.is_atom) {
            pending.push_back(elem.atom);
        } else {
            throw std::runtime_error("PDDL: malformed " + context + ": expected atom");
        }
    }
    // Remaining items without explicit type get "object"
    for (auto& name : pending) {
        result.push_back({name, "object"});
    }
    return result;
}

static bool isContextualExpressionHead(const std::string& head) {
    static const std::vector<std::string> contextual = {"and", "not"};
    return std::find(contextual.begin(), contextual.end(), head) != contextual.end();
}

static bool isUnsupportedExpressionHead(const std::string& head) {
    static const std::vector<std::string> unsupported = {
        "or", "forall", "exists", "=", "when", "imply", "=>",
        "increase", "decrease", "assign", "scale-up", "scale-down",
        ">", "<", ">=", "<=", "+", "-", "*", "/"
    };
    return std::find(unsupported.begin(), unsupported.end(), head) != unsupported.end();
}

static bool isNonPredicateExpressionHead(const std::string& head) {
    return isContextualExpressionHead(head) || isUnsupportedExpressionHead(head);
}

static std::string expressionHead(const SExpr& expr) {
    if (expr.is_atom) {
        return expr.atom;
    }
    if (expr.children.empty()) {
        return "<empty>";
    }
    if (!expr.children[0].is_atom) {
        return "<compound>";
    }
    return expr.children[0].atom;
}

// Parse a predicate atom like (at ?r ?loc) from an S-expression
// Returns: predicate name, list of argument names
static std::pair<std::string, std::vector<std::string>> parseAtom(
    const SExpr& expr,
    const std::string& context)
{
    if (expr.is_atom) {
        // 0-ary predicate
        return {expr.atom, {}};
    }
    if (expr.children.empty() || !expr.children[0].is_atom) {
        throw std::runtime_error("PDDL: malformed atom in " + context);
    }
    std::string name = expr.children[0].atom;
    if (isNonPredicateExpressionHead(name)) {
        throw std::runtime_error("PDDL: expression head '" + name +
                                 "' cannot be used as an atom in " + context);
    }
    std::vector<std::string> args;
    for (size_t i = 1; i < expr.children.size(); ++i) {
        if (expr.children[i].is_atom) {
            args.push_back(expr.children[i].atom);
        } else {
            throw std::runtime_error("PDDL: malformed atom in " + context +
                                     ": nested expression under predicate '" + name + "'");
        }
    }
    return {name, args};
}

static void validateRequirements(const SExpr& section, const std::string& context) {
    static const std::vector<std::string> supported = {":strips", ":typing"};
    for (size_t i = 1; i < section.children.size(); ++i) {
        if (!section.children[i].is_atom) {
            throw std::runtime_error("PDDL: malformed :requirements block in " + context);
        }
        const std::string& flag = section.children[i].atom;
        if (std::find(supported.begin(), supported.end(), flag) == supported.end()) {
            throw std::runtime_error("PDDL: unsupported requirement '" + flag + "' in " + context);
        }
    }
}

// =========================================================================
// Domain parsing
// =========================================================================

struct PddlAction {
    std::string name;
    std::vector<std::string> param_names;
    std::vector<std::string> param_types;   // may contain "a|b" union (either) tokens
    SExpr precondition_expr;                 // raw precondition, normalized later
    bool has_precondition = false;
    std::vector<std::string> add_atoms;
    std::vector<std::string> del_atoms;
};

// One disjunct of a normalized precondition (disjunctive normal form): a
// conjunction of positive/negative atoms plus equality binding filters.
struct PreClause {
    std::vector<std::string> pos;             // positive atom templates
    std::vector<std::string> neg;             // negative atom templates
    std::vector<EqualityConstraint> eq;       // equality/inequality filters
};

// Upper bound on DNF expansion to keep disjunction / existential blow-up
// bounded; exceeding it fails closed rather than emitting a huge schema set.
static constexpr size_t kMaxPreClauses = 4096;

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

// Parse a conjunction of (possibly negated) atoms.
// Positive atoms go to `atoms`. Negative atoms `(not ATOM)` go to `neg_atoms`,
// when a negative sink is provided; passing neg_atoms == nullptr (e.g. for
// goals) rejects negation entirely so goals remain positive-only.
// Only `(not ATOM)` is accepted as a negative; nested negation, `(not (and ...))`,
// and `(not (or ...))` are rejected because parseAtom refuses non-predicate heads.
static void parsePrecondition(const SExpr& expr,
                              std::vector<std::string>& atoms,
                              std::vector<std::string>* neg_atoms,
                              const std::string& context) {
    if (expr.is_atom) {
        auto [pred, args] = parseAtom(expr, context);
        atoms.push_back(buildFluentTemplate(pred, args));
        return;
    }
    if (expr.children.empty()) return;
    if (!expr.children[0].is_atom) {
        throw std::runtime_error("PDDL: malformed precondition in " + context);
    }

    const std::string head = expr.children[0].atom;
    if (head == "and") {
        for (size_t i = 1; i < expr.children.size(); ++i) {
            parsePrecondition(expr.children[i], atoms, neg_atoms, context);
        }
    } else if (head == "not") {
        if (neg_atoms == nullptr) {
            throw std::runtime_error("PDDL: unsupported construct 'not' in " +
                                     context + ": negation is not allowed here");
        }
        if (expr.children.size() != 2) {
            throw std::runtime_error("PDDL: malformed negative precondition in " +
                                     context + ": expected (not ATOM)");
        }
        // parseAtom rejects compound heads (and/or/not/...), so (not (and ...))
        // and nested negation fail closed here.
        auto [pred, args] = parseAtom(expr.children[1],
                                      "negative precondition in " + context);
        neg_atoms->push_back(buildFluentTemplate(pred, args));
    } else if (isUnsupportedExpressionHead(head)) {
        throw std::runtime_error("PDDL: unsupported construct '" + head +
                                 "' in precondition in " + context);
    } else {
        auto [pred, args] = parseAtom(expr, context);
        atoms.push_back(buildFluentTemplate(pred, args));
    }
}

// Parse effect (handles bare atom, (and ...), (not ...))
static void parseEffect(const SExpr& expr,
                        std::vector<std::string>& add_atoms,
                        std::vector<std::string>& del_atoms,
                        const std::string& context) {
    if (expr.is_atom) {
        auto [pred, args] = parseAtom(expr, context);
        add_atoms.push_back(buildFluentTemplate(pred, args));
        return;
    }
    if (expr.children.empty()) return;
    if (!expr.children[0].is_atom) {
        throw std::runtime_error("PDDL: malformed effect in " + context);
    }

    const std::string head = expr.children[0].atom;
    if (head == "and") {
        for (size_t i = 1; i < expr.children.size(); ++i) {
            parseEffect(expr.children[i], add_atoms, del_atoms, context);
        }
    } else if (head == "not") {
        if (expr.children.size() != 2) {
            throw std::runtime_error("PDDL: malformed delete effect in " + context);
        }
        auto [pred, args] = parseAtom(expr.children[1], "delete effect in " + context);
        del_atoms.push_back(buildFluentTemplate(pred, args));
    } else if (isUnsupportedExpressionHead(head)) {
        throw std::runtime_error("PDDL: unsupported construct '" + head +
                                 "' in effect in " + context);
    } else {
        auto [pred, args] = parseAtom(expr, context);
        add_atoms.push_back(buildFluentTemplate(pred, args));
    }
}

// =========================================================================
// Precondition normalization to disjunctive normal form (DNF)
// =========================================================================
// Lowers a precondition S-expression into a disjunction of PreClauses (each a
// conjunction of positive/negative atoms + equality filters). This single pass
// handles nested `and`, `or`, `(not ATOM)`, `(= a b)` / `(not (= a b))`, and
// finite `forall` / `exists` quantifiers (expanded over the known object set).

// Replace every atom equal to `var` with `obj` throughout an S-expression.
static SExpr substituteVarInSExpr(const SExpr& expr,
                                  const std::string& var,
                                  const std::string& obj) {
    SExpr out = expr;
    if (out.is_atom) {
        if (out.atom == var) out.atom = obj;
        return out;
    }
    for (auto& child : out.children) {
        child = substituteVarInSExpr(child, var, obj);
    }
    return out;
}

// Cartesian AND of two DNF lists: every pair of clauses is concatenated.
static std::vector<PreClause> andCombine(const std::vector<PreClause>& a,
                                         const std::vector<PreClause>& b,
                                         const std::string& context) {
    std::vector<PreClause> result;
    if (a.empty() || b.empty()) return result;  // false AND anything == false
    if (a.size() * b.size() > kMaxPreClauses) {
        throw std::runtime_error("PDDL: precondition in " + context +
                                 " expands to too many disjuncts (limit " +
                                 std::to_string(kMaxPreClauses) + ")");
    }
    result.reserve(a.size() * b.size());
    for (const auto& ca : a) {
        for (const auto& cb : b) {
            PreClause merged = ca;
            merged.pos.insert(merged.pos.end(), cb.pos.begin(), cb.pos.end());
            merged.neg.insert(merged.neg.end(), cb.neg.begin(), cb.neg.end());
            merged.eq.insert(merged.eq.end(), cb.eq.begin(), cb.eq.end());
            result.push_back(std::move(merged));
        }
    }
    return result;
}

static std::vector<PreClause> normalizePrecondition(
    const SExpr& expr,
    const TypeSystem& types,
    const std::vector<std::string>& param_names,
    const std::string& context);

// Expand a finite quantifier body over all instantiations of its typed
// variables. `is_forall` selects AND-combination (universal) vs OR-union
// (existential).
static std::vector<PreClause> expandQuantifier(
    const SExpr& expr,
    const TypeSystem& types,
    const std::vector<std::string>& param_names,
    const std::string& context,
    bool is_forall) {
    const std::string kw = is_forall ? "forall" : "exists";
    if (expr.children.size() != 3 || expr.children[1].is_atom) {
        throw std::runtime_error("PDDL: malformed " + kw + " in " + context +
                                 ": expected (" + kw + " (vars) body)");
    }
    auto vars = parseTypedList(expr.children[1].children, 0,
                               kw + " variables in " + context);
    for (const auto& [var, type] : vars) {
        (void)type;
        if (std::find(param_names.begin(), param_names.end(), var) != param_names.end()) {
            throw std::runtime_error("PDDL: " + kw + " variable '" + var +
                                     "' in " + context + " shadows an action parameter");
        }
    }

    // Build the cartesian product of object instantiations for the variables.
    std::vector<std::vector<std::string>> instantiations{{}};
    for (const auto& [var, type] : vars) {
        (void)var;
        std::vector<std::vector<std::string>> next;
        for (const auto& obj : types.getObjectsOfType(type)) {
            for (const auto& partial : instantiations) {
                auto extended = partial;
                extended.push_back(obj);
                next.push_back(std::move(extended));
            }
        }
        instantiations = std::move(next);
    }

    // Universal over an empty domain is vacuously true (one empty clause);
    // existential over an empty domain is unsatisfiable (no clauses).
    std::vector<PreClause> result;
    if (is_forall) result.push_back(PreClause{});

    for (const auto& inst : instantiations) {
        SExpr body = expr.children[2];
        for (size_t k = 0; k < vars.size(); ++k) {
            body = substituteVarInSExpr(body, vars[k].first, inst[k]);
        }
        auto body_clauses = normalizePrecondition(body, types, param_names, context);
        if (is_forall) {
            result = andCombine(result, body_clauses, context);
        } else {
            result.insert(result.end(), body_clauses.begin(), body_clauses.end());
            if (result.size() > kMaxPreClauses) {
                throw std::runtime_error("PDDL: exists in " + context +
                                         " expands to too many disjuncts");
            }
        }
    }
    return result;
}

static std::vector<PreClause> normalizePrecondition(
    const SExpr& expr,
    const TypeSystem& types,
    const std::vector<std::string>& param_names,
    const std::string& context) {
    // Bare atom -> single positive literal.
    if (expr.is_atom) {
        auto [pred, args] = parseAtom(expr, context);
        PreClause c;
        c.pos.push_back(buildFluentTemplate(pred, args));
        return {c};
    }
    if (expr.children.empty()) {
        // Empty conjunction: vacuously true.
        return {PreClause{}};
    }
    if (!expr.children[0].is_atom) {
        throw std::runtime_error("PDDL: malformed precondition in " + context);
    }

    const std::string head = expr.children[0].atom;

    if (head == "and") {
        std::vector<PreClause> result{PreClause{}};
        for (size_t i = 1; i < expr.children.size(); ++i) {
            result = andCombine(
                result,
                normalizePrecondition(expr.children[i], types, param_names, context),
                context);
        }
        return result;
    }
    if (head == "or") {
        std::vector<PreClause> result;
        for (size_t i = 1; i < expr.children.size(); ++i) {
            auto sub = normalizePrecondition(expr.children[i], types, param_names, context);
            result.insert(result.end(), sub.begin(), sub.end());
            if (result.size() > kMaxPreClauses) {
                throw std::runtime_error("PDDL: or in " + context +
                                         " expands to too many disjuncts");
            }
        }
        return result;  // empty (or) -> unsatisfiable
    }
    if (head == "not") {
        if (expr.children.size() != 2) {
            throw std::runtime_error("PDDL: malformed negation in " + context +
                                     ": expected (not ATOM) or (not (= a b))");
        }
        const SExpr& child = expr.children[1];
        // (not (= a b)) -> inequality binding filter.
        if (!child.is_atom && !child.children.empty() &&
            child.children[0].is_atom && child.children[0].atom == "=") {
            if (child.children.size() != 3 ||
                !child.children[1].is_atom || !child.children[2].is_atom) {
                throw std::runtime_error("PDDL: malformed (not (= a b)) in " + context);
            }
            PreClause c;
            c.eq.push_back({child.children[1].atom, child.children[2].atom, false});
            return {c};
        }
        // (not ATOM) -> negative literal. parseAtom rejects compound heads, so
        // (not (and ...)) / nested negation fail closed.
        auto [pred, args] = parseAtom(child, "negative precondition in " + context);
        PreClause c;
        c.neg.push_back(buildFluentTemplate(pred, args));
        return {c};
    }
    if (head == "=") {
        if (expr.children.size() != 3 ||
            !expr.children[1].is_atom || !expr.children[2].is_atom) {
            throw std::runtime_error("PDDL: malformed (= a b) in " + context);
        }
        PreClause c;
        c.eq.push_back({expr.children[1].atom, expr.children[2].atom, true});
        return {c};
    }
    if (head == "forall") {
        return expandQuantifier(expr, types, param_names, context, /*is_forall=*/true);
    }
    if (head == "exists") {
        return expandQuantifier(expr, types, param_names, context, /*is_forall=*/false);
    }
    if (isUnsupportedExpressionHead(head)) {
        throw std::runtime_error("PDDL: unsupported construct '" + head +
                                 "' in precondition in " + context);
    }
    // Ordinary predicate atom.
    auto [pred, args] = parseAtom(expr, context);
    PreClause c;
    c.pos.push_back(buildFluentTemplate(pred, args));
    return {c};
}

static PddlAction parseAction(const SExpr& expr) {
    PddlAction act;

    // (:action name :parameters (...) :precondition (...) :effect (...))
    size_t i = 1; // skip ":action"
    if (i < expr.children.size() && expr.children[i].is_atom) {
        act.name = expr.children[i].atom;
        ++i;
    } else {
        throw std::runtime_error("PDDL: malformed :action without name");
    }
    const std::string context = "action '" + act.name + "'";

    while (i < expr.children.size()) {
        if (expr.children[i].is_atom) {
            const std::string& kw = expr.children[i].atom;
            ++i;
            if (i >= expr.children.size()) {
                throw std::runtime_error("PDDL: missing value for " + kw + " in " + context);
            }

            if (kw == ":parameters") {
                // Next is a list of typed params
                auto& params_expr = expr.children[i];
                if (params_expr.is_atom) {
                    throw std::runtime_error("PDDL: malformed :parameters in " + context);
                }
                auto typed = parseTypedList(params_expr.children, 0,
                                            "parameters in " + context,
                                            /*allow_either=*/true);
                for (auto& [name, type] : typed) {
                    act.param_names.push_back(name);
                    act.param_types.push_back(type);
                }
                ++i;
            } else if (kw == ":precondition") {
                // Store the raw precondition; it is normalized to DNF at
                // registration time, once the object set is known.
                act.precondition_expr = expr.children[i];
                act.has_precondition = true;
                ++i;
            } else if (kw == ":effect") {
                parseEffect(expr.children[i], act.add_atoms, act.del_atoms, context);
                ++i;
            } else {
                throw std::runtime_error("PDDL: unsupported action field '" + kw +
                                         "' in " + context);
            }
        } else {
            throw std::runtime_error("PDDL: malformed action body in " + context +
                                     ": expected keyword, got '" +
                                     expressionHead(expr.children[i]) + "'");
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

// Returns true when `name` is already declared with exactly `type`, so the
// caller should skip re-adding it.
//
// A WorldModel is a session, not a one-shot parse target: a host that re-grounds
// a new problem into a live session legitimately re-declares the same entities,
// zones, and capabilities every time. Rejecting that made re-tasking impossible
// -- the second parse died on the first shared object name.
//
// The diagnostic this check exists for is a name that means two different
// things, and that is still rejected, now saying which two types disagreed
// instead of only that a name repeated.
static bool declareObject(WorldModel& wm,
                          const std::string& name,
                          const std::string& type,
                          const std::string& context) {
    if (!wm.typeSystem().hasObject(name)) return false;
    const std::string existing = wm.typeSystem().getObjectType(name);
    if (existing != type) {
        throw std::runtime_error("PDDL: " + context + " '" + name +
                                 "' is declared as type '" + type +
                                 "' but already exists as type '" + existing +
                                 "'. Each object has exactly one type.");
    }
    return true;
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

    auto* domain_requirements = findSection(domain, ":requirements");
    if (domain_requirements) {
        validateRequirements(*domain_requirements, "domain");
    }

    auto* problem_requirements = findSection(problem, ":requirements");
    if (problem_requirements) {
        validateRequirements(*problem_requirements, "problem");
    }

    // Types
    auto* types_section = findSection(domain, ":types");
    if (types_section) {
        auto typed = parseTypedList(types_section->children, 1, "domain types");
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

    // Constants
    auto* constants_section = findSection(domain, ":constants");
    if (constants_section) {
        auto typed = parseTypedList(constants_section->children, 1, "domain constants");
        for (auto& [name, type] : typed) {
            if (declareObject(wm, name, type, "domain constant")) continue;
            wm.addObject(name, type);
        }
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
                if (!pred_expr.children[0].is_atom) {
                    throw std::runtime_error("PDDL: malformed predicate declaration");
                }
                std::string pred_name = pred_expr.children[0].atom;
                auto typed = parseTypedList(pred_expr.children, 1,
                                            "predicate '" + pred_name + "'");
                std::vector<std::string> param_types;
                for (auto& [name, type] : typed) {
                    param_types.push_back(type);
                }
                wm.registerPredicate(pred_name, param_types);
            }
        }
    }

    // Predicates the domain declares as evidence-bearing: a precondition on one
    // is met only by an observed fact, never by a value a plan effect predicted.
    // Declared per predicate so one action can mix predicted and observed
    // preconditions, e.g. a strike that is predicted to be airborne but must be
    // observed to be authorised.
    //
    //   (:confirmed-predicates authorised route-clear)
    auto* confirmed_section = findSection(domain, ":confirmed-predicates");
    if (confirmed_section) {
        for (size_t i = 1; i < confirmed_section->children.size(); ++i) {
            auto& entry = confirmed_section->children[i];
            if (entry.is_atom) {
                wm.registerConfirmedPredicate(entry.atom);
            } else if (!entry.children.empty() && entry.children[0].is_atom) {
                // Tolerate a parenthesised form, e.g. (authorised).
                wm.registerConfirmedPredicate(entry.children[0].atom);
            } else {
                throw std::runtime_error(
                    "PDDL: malformed :confirmed-predicates entry");
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
        auto typed = parseTypedList(obj_section->children, 1, "problem objects");
        for (auto& [name, type] : typed) {
            if (declareObject(wm, name, type, "problem object")) continue;
            wm.addObject(name, type);
        }
    }

    // Now register actions (after objects so grounding and quantifier/either
    // expansion can see the full object set).
    for (auto& act : actions) {
        // Validate any (either ...) union parameter types refer to known types.
        for (const auto& ptype : act.param_types) {
            if (ptype.find('|') == std::string::npos) continue;
            size_t start = 0;
            while (start <= ptype.size()) {
                size_t bar = ptype.find('|', start);
                std::string t = ptype.substr(
                    start, bar == std::string::npos ? std::string::npos : bar - start);
                if (!t.empty() && !wm.typeSystem().hasType(t)) {
                    throw std::runtime_error("PDDL: unknown type '" + t +
                                             "' in (either ...) of action '" +
                                             act.name + "'");
                }
                if (bar == std::string::npos) break;
                start = bar + 1;
            }
        }

        // Normalize the precondition into DNF: one schema per disjunct.
        std::vector<PreClause> clauses;
        if (act.has_precondition) {
            clauses = normalizePrecondition(act.precondition_expr, wm.typeSystem(),
                                            act.param_names, "action '" + act.name + "'");
        } else {
            clauses.push_back(PreClause{});  // no precondition == always applicable
        }

        // Empty DNF (e.g. an `(or)` or `exists` over an empty domain) means the
        // precondition is unsatisfiable: the action can never apply, so it
        // grounds to no schema.
        if (clauses.size() == 1) {
            wm.registerAction(act.name, act.param_names, act.param_types,
                              clauses[0].pos, clauses[0].neg,
                              act.add_atoms, act.del_atoms, clauses[0].eq);
        } else {
            // Multiple disjuncts: register one schema each, tagging the name with
            // a disjunct index so ground-action signatures stay distinct. The
            // BT compiler strips the "#k" tag to resolve the shared implementation.
            for (size_t k = 0; k < clauses.size(); ++k) {
                wm.registerAction(act.name + "#" + std::to_string(k),
                                  act.param_names, act.param_types,
                                  clauses[k].pos, clauses[k].neg,
                                  act.add_atoms, act.del_atoms, clauses[k].eq);
            }
        }
    }

    // Init
    auto* init_section = findSection(problem, ":init");
    if (init_section) {
        for (size_t i = 1; i < init_section->children.size(); ++i) {
            auto [pred, args] = parseAtom(init_section->children[i], "initial state");
            std::string key = buildFluentTemplate(pred, args);
            wm.setFact(key, true);
        }
    }

    // Goal
    auto* goal_section = findSection(problem, ":goal");
    if (goal_section && goal_section->children.size() >= 2) {
        const SExpr& goal_expr = goal_section->children[1];
        const bool is_disjunctive =
            !goal_expr.is_atom && !goal_expr.children.empty() &&
            goal_expr.children[0].is_atom && goal_expr.children[0].atom == "or";

        if (is_disjunctive) {
            // (or A B ...) -> one positive goal alternative per disjunct; any
            // one satisfied counts as success.
            std::vector<std::vector<std::string>> alternatives;
            for (size_t i = 1; i < goal_expr.children.size(); ++i) {
                std::vector<std::string> alt_atoms;
                // Each alternative stays positive-only (nullptr rejects negation).
                parsePrecondition(goal_expr.children[i], alt_atoms, nullptr, "goal");
                alternatives.push_back(std::move(alt_atoms));
            }
            if (alternatives.empty()) {
                throw std::runtime_error("PDDL: empty (or) in goal");
            }
            wm.setGoalAlternatives(alternatives);
        } else {
            std::vector<std::string> goal_atoms;
            // Goals stay positive-only: pass nullptr so negation is rejected.
            parsePrecondition(goal_expr, goal_atoms, nullptr, "goal");
            wm.setGoal(goal_atoms);
        }
    }
}

} // namespace ame
