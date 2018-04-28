// This class is used for parsing simple text math expressions

#ifndef ExpressionParser_h
#define ExpressionParser_h

#include <vector>
#include <string>
#include <map>
#include "PGDMath.h"

struct ExpressionParserToken;
class ExpressionFolder;

enum ExpressionParserTokenType
{
    EPError = 0,
    EPFunction = 1,
    EPOperator = 2,
    EPNumber = 3,
    EPVariable = 4,
    EPParser = 5,
    EPUnaryOperator = 6,
    EPVector = 7,
    EPMatrix3x3 = 8,
    EPNumberList = 9,
    EPPostfixOperator = 10,
    EPMatrix3x3Val = 11,
    EPVectorVal = 12
};

struct ExpressionParserValue
{
    ExpressionParserValue(ExpressionParserTokenType tokenType = EPError) { type = tokenType; };

    ExpressionParserTokenType type;
    double value;
    int op;
    pgd::Matrix3x3 matrix;
    pgd::Vector vector;
};

class ExpressionParser
{
public:
    ExpressionParser();
    ExpressionParser(ExpressionParser &expressionParser);
    ExpressionParser(ExpressionParser *expressionParser); // this version does NOT copy the whole ExpressionParser

    ~ExpressionParser();

    int CreateFromString(const std::string &input);
    int CreateFromString(char *input, int length);
    ExpressionParserValue Evaluate();

    void SetFolder(ExpressionFolder *folder) { m_Folder = folder; };

    void SetVData(unsigned int size, double *data);

    static void Tokenizer(const char *constbuf, std::vector<std::string> &tokens, const char *stopList = 0);

protected:

    ExpressionParserValue EvaluateParser(ExpressionParser *parser);
    ExpressionParserValue EvaluateToken(ExpressionParserToken *token);
    ExpressionParserValue EvaluateVariable(ExpressionParserToken *token);
    ExpressionParserValue EvaluateFunction(ExpressionParserToken *token);
    ExpressionParserValue EvaluateOperator(ExpressionParserValue lhs, ExpressionParserValue rhs, int op);
    ExpressionParserValue EvaluateVector(ExpressionParserToken *token);
    ExpressionParserValue EvaluateMatrix3x3(ExpressionParserToken *token);
    ExpressionParserValue EvaluateNumberList(ExpressionParserToken *token);
    ExpressionParserValue EvaluateGlobalFunction(ExpressionParserToken *token);
    ExpressionParserValue EvaluateGlobalVariable(ExpressionParserToken *token);

    bool VariableStartTest(const char *p);
    bool VariableStartCharacter(int c);
    bool VariableMiddleCharacter(int c);

    std::vector<ExpressionParserToken *> m_ExpressionParserTokenList;
    ExpressionFolder *m_Folder;

    // v data (this is a bit of a hack)
    std::vector<double> m_v;
};

struct ExpressionParserToken
{
public:
    ExpressionParserToken() {};
    ExpressionParserToken(ExpressionParserToken &epToken)
    {
        token = epToken.token;
        name = epToken.name;
        for (unsigned int i = 0; i < epToken.parserList.size(); i++)
        {
            parserList.push_back(new ExpressionParser(*epToken.parserList[i]));
        }
    }

    ~ExpressionParserToken() { for (unsigned int i = 0; i < parserList.size(); i++) delete parserList[i]; };

    ExpressionParserValue token;
    std::string name;
    std::vector<ExpressionParser *> parserList;
};



#endif

