// This class is used for parsing simple text math expressions

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <cmath>

#include "ExpressionParser.h"
#include "PGDMath.h"
#include "ExpressionVar.h"
#include "ExpressionVec.h"
#include "ExpressionMat.h"
#include "ExpressionRef.h"
#include "ExpressionFunTransform3D.h"

#include "DebugControl.h"

ExpressionParser::ExpressionParser()
{
    m_Folder = 0;
}

ExpressionParser::ExpressionParser(ExpressionParser &expressionParser)
{
    m_Folder = expressionParser.m_Folder;
    m_v = expressionParser.m_v;
    ExpressionParserToken *newToken;
    for (unsigned int i = 0; i < expressionParser.m_ExpressionParserTokenList.size(); i++)
    {
        newToken = new ExpressionParserToken(*expressionParser.m_ExpressionParserTokenList[i]);
        m_ExpressionParserTokenList.push_back(newToken);
    }
}

// this version ONLY copies the environment list
ExpressionParser::ExpressionParser(ExpressionParser *expressionParser)
{
    m_Folder = expressionParser->m_Folder;
}

ExpressionParser::~ExpressionParser()
{
    for (unsigned int i = 0; i < m_ExpressionParserTokenList.size(); i++)
        delete m_ExpressionParserTokenList[i];
}

// initialise the ExpressionParser based on a string
// returns 0 if successful
int ExpressionParser::CreateFromString(const std::string &input)
{
    char *buf = new char[input.size() + 1];
    strcpy(buf, input.c_str());
    return CreateFromString(buf, input.size());
}


// initialise the ExpressionParser based on a string
// returns 0 if successful
// the character input[length] must exist (can be '\0')
int ExpressionParser::CreateFromString(char *input, int length)
{
    int index = 0;
    int parenthesisStart;
    int parenthesisCount;
    int braceCount;
    int functionStart;
    ExpressionParserToken *expressionParserToken;
    bool finished;
    char byte;
    char *ptr;
    int j;
    bool expectingNumber = true;
    ExpressionParser *expressionParser;
    int nargs;
    int err;

    if (m_ExpressionParserTokenList.size() > 0)
    {
        unsigned int ii;
        for (ii = 0; ii < m_ExpressionParserTokenList.size(); ii++)
            delete m_ExpressionParserTokenList[ii];
        m_ExpressionParserTokenList.clear();
    }

    while (index < length)
    {
        if (isspace(input[index]) == 0) // not white space
        {
            if (input[index] == '(') // left parenthesis - need to find matching ')' and recurse
            {
                if (expectingNumber == false)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Not expecting a number\n" << input << "\n";
                    return __LINE__;
                }
                expectingNumber = false;
                finished = false;
                index++;
                parenthesisStart = index;
                parenthesisCount = 1;
                while (finished == false)
                {
                    if (input[index] == '(') parenthesisCount++;
                    else if (input[index] == ')') parenthesisCount--;
                    if (parenthesisCount == 0)
                    {
                        expressionParserToken = new ExpressionParserToken();
                        m_ExpressionParserTokenList.push_back(expressionParserToken);
                        expressionParserToken->token.type = EPParser;
                        expressionParser = new ExpressionParser(this);
                        expressionParserToken->parserList.push_back(expressionParser);
                        err = expressionParser->CreateFromString(&input[parenthesisStart], index - parenthesisStart);
                        if (err) return err;
                        finished = true;
                    }
                    index++;
                    if (index >= length && finished == false)
                    {
                        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unmatched '('\n" << input << "\n";
                        return __LINE__;
                    }
                }
            }

            else if (VariableStartTest(&input[index])) // variable or function
            {
                if (expectingNumber == false)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Not expecting a number\n" << input << "\n";
                    return __LINE__;
                }
                expectingNumber = false;
                finished = false;
                expressionParserToken = new ExpressionParserToken();
                m_ExpressionParserTokenList.push_back(expressionParserToken);
                expressionParserToken->token.type = EPVariable;
                expressionParserToken->name = input[index];
                while (finished == false)
                {
                    index++;
                    if (index >= length) finished = true;
                    else if (VariableMiddleCharacter(input[index]) == 0) finished = true;
                    else expressionParserToken->name += input[index];
                }
                if (index < length)
                {
                    if (input[index] == '(') // got a function
                    {
                        expressionParserToken->token.type = EPFunction;
                        index++;
                        finished = false;
                        while (finished == false)
                        {
                            functionStart = index;
                            parenthesisCount = 0;
                            braceCount = 0;
                            while (finished == false)
                            {
                                if (index >= length)
                                {
                                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Cannot find ','\n" << input << "\n";
                                    return __LINE__;
                                }
                                if (input[index] == '(') parenthesisCount++;
                                else if (input[index] == ')') parenthesisCount--;
                                else if (input[index] == '{') braceCount++;
                                else if (input[index] == '}') braceCount--;
                                if (parenthesisCount == 0)
                                {
                                    if (input[index] == ',' && braceCount == 0)
                                    {
                                        expressionParser = new ExpressionParser(this);
                                        expressionParserToken->parserList.push_back(expressionParser);
                                        expressionParser->CreateFromString(&input[functionStart], index - functionStart);
                                        break;
                                    }
                                }
                                else if (parenthesisCount == -1)
                                {
                                    expressionParser = new ExpressionParser(this);
                                    expressionParserToken->parserList.push_back(expressionParser);
                                    expressionParser->CreateFromString(&input[functionStart], index - functionStart);
                                    finished = true;
                                    break;
                                }
                                index++;
                            }
                            index++;
                        }
                    }
                    else if (input[index] == '[') // got an array
                    {
                        // check in m_VectorList
                        ExpressionVec *p2 = dynamic_cast<ExpressionVec *>(m_Folder->Search(expressionParserToken->name, true));
                        if (p2)
                        {
                            nargs = 1;
                            expressionParserToken->token.type = EPVector;
                        }
                        else
                        {
                            ExpressionMat *p3 = dynamic_cast<ExpressionMat *>(m_Folder->Search(expressionParserToken->name, true));
                            if (p3)
                            {
                                nargs = 2;
                                expressionParserToken->token.type = EPMatrix3x3;
                            }
                            else
                            {
                                std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Cannot find:" << expressionParserToken->name << "\n";
                                return __LINE__;
                            }
                        }
                        for (j = 0; j < nargs; j++)
                        {
                            while (input[index] <= ' ')
                            {
                                index++;
                                if (index >= length)
                                {
                                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Cannot find '['\n" << input << "\n";
                                    return __LINE__;
                                }
                            }
                            if (input[index] != '[')
                            {
                                std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Expected '['\n" << input << "\n";
                                return __LINE__;
                            }
                            index++;
                            finished = false;
                            functionStart = index;
                            parenthesisCount = 0;
                            while (finished == false)
                            {
                                if (index >= length)
                                {
                                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Cannot find ']'\n" << input << "\n";
                                    return __LINE__;
                                }
                                if (input[index] == '[') parenthesisCount++;
                                else if (input[index] == ']') parenthesisCount--;
                                if (parenthesisCount == -1)
                                {
                                    expressionParser = new ExpressionParser(this);
                                    expressionParserToken->parserList.push_back(expressionParser);
                                    expressionParser->CreateFromString(&input[functionStart], index - functionStart);
                                    finished = true;
                                }
                                index++;
                            }

                        }
                    }

                }
            }

            else if (input[index] == '{' ) // array constant
            {
                if (expectingNumber == false)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Not expecting a number\n" << input << "\n";
                    return __LINE__;
                }
                expectingNumber = false;
                expressionParserToken = new ExpressionParserToken();
                m_ExpressionParserTokenList.push_back(expressionParserToken);
                expressionParserToken->token.type = EPNumberList;
                index++;
                finished = false;
                while (finished == false)
                {
                    functionStart = index;
                    parenthesisCount = 0;
                    while (finished == false)
                    {
                        if (index >= length)
                        {
                            std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Cannot find ','\n" << input << "\n";
                            return __LINE__;
                        }
                        if (input[index] == '{') parenthesisCount++;
                        else if (input[index] == '}') parenthesisCount--;
                        if (parenthesisCount == 0)
                        {
                            if (input[index] == ',')
                            {
                                expressionParser = new ExpressionParser(this);
                                expressionParserToken->parserList.push_back(expressionParser);
                                expressionParser->CreateFromString(&input[functionStart], index - functionStart);
                                break;
                            }
                        }
                        else if (parenthesisCount == -1)
                        {
                            expressionParser = new ExpressionParser(this);
                            expressionParserToken->parserList.push_back(expressionParser);
                            expressionParser->CreateFromString(&input[functionStart], index - functionStart);
                            finished = true;
                            break;
                        }
                        index++;
                    }
                    index++;
                }
            }

            else if ((expectingNumber && strchr("01234567890.", input[index]))) // number
            {
                if (expectingNumber == false)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Not expecting a number\n" << input << "\n";
                    return __LINE__;
                }
                expectingNumber = false;
                expressionParserToken = new ExpressionParserToken();
                m_ExpressionParserTokenList.push_back(expressionParserToken);
                expressionParserToken->token.type = EPNumber;
                // kludge so I can use strtod
                byte = input[length];
                input[length] = 0;
                expressionParserToken->token.value = strtod(&input[index], &ptr);
                if (ptr == &input[index])
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Could not convert number\n" << input << "\n";
                    input[length] = byte;
                    return __LINE__;
                }
                index += (int)(ptr - &input[index]);
                input[length] = byte;
            }

            else if (expectingNumber && (input[index] == '+' || input[index] == '-')) // unary operator
            {
                if (input[index] == '-') // unary operator + doesn't do anything
                {
                    expressionParserToken = new ExpressionParserToken();
                    m_ExpressionParserTokenList.push_back(expressionParserToken);
                    expressionParserToken->token.type = EPUnaryOperator;
                    expressionParserToken->token.op = input[index];
                }
                index++;
            }

            else if (input[index] == '\'' ) // postfix operator
            {
                if (expectingNumber == true)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Not expecting a postfix operator\n" << input << "\n";
                    return __LINE__;
                }
                expressionParserToken = new ExpressionParserToken();
                m_ExpressionParserTokenList.push_back(expressionParserToken);
                expressionParserToken->token.type = EPPostfixOperator;
                expressionParserToken->token.op = input[index];
                index++;
            }

            else if (strchr("+-/*!><=|&^", input[index])) // operator
            {
                if (expectingNumber == true)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "ParseError: Not expecting an operator\n" << input << "\n";
                    return __LINE__;
                }
                expectingNumber = true;
                expressionParserToken = new ExpressionParserToken();
                m_ExpressionParserTokenList.push_back(expressionParserToken);
                expressionParserToken->token.type = EPOperator;
                switch(input[index])
                {
                    case '+':
                    case '-':
                    case '/':
                    case '*':
                    case '&':
                    case '|':
                    case '=':
                    case '!':
                    case '^':
                        expressionParserToken->token.op = input[index];
                        index++;
                        break;

                    case '<':
                    case '>':
                        expressionParserToken->token.op = input[index];
                        index++;
                        if (index >= length) break;
                            if (input[index] == '=')
                            {
                                index++;
                                expressionParserToken->token.op += 256;
                            }
                                break;

                }
             }

             else
             {
                 std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unrecognised element\n" << input << "\n";
                 return __LINE__;
             }
        }
    }

    return 0;
}

// Evaluate expression from genome
ExpressionParserValue ExpressionParser::Evaluate()
{
    return EvaluateParser(this);
}

// Evaluate expression
ExpressionParserValue ExpressionParser::EvaluateParser(ExpressionParser *parser)
{
    unsigned int i;
    ExpressionParserToken *token;
    std::vector<ExpressionParserToken *>::iterator iter1;
    std::vector<ExpressionParserToken *>::iterator iter2;

    if (parser->m_ExpressionParserTokenList.size() == 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: No tokens\n";
        return ExpressionParserValue(EPError);
    }

    // variables, functions and arrays
    for (i = 0; i < parser->m_ExpressionParserTokenList.size(); i ++)
    {
        token = parser->m_ExpressionParserTokenList[i];
        if (token->token.type == EPParser ||
            token->token.type == EPFunction ||
            token->token.type == EPVariable ||
            token->token.type == EPVector ||
            token->token.type == EPMatrix3x3 ||
            token->token.type == EPNumberList )
        {
            EvaluateToken(token);
        }
    }

    // unary operators - can only be minus or '

    for (iter1 = parser->m_ExpressionParserTokenList.begin(); iter1 != parser->m_ExpressionParserTokenList.end(); iter1++)
    {
        if ((*iter1)->token.type == EPUnaryOperator)
        {
            delete *iter1;
            iter1 = parser->m_ExpressionParserTokenList.erase(iter1);
            if ((*iter1)->token.type == EPNumber) (*iter1)->token.value = -1 * (*iter1)->token.value;
            else if ((*iter1)->token.type == EPVectorVal) (*iter1)->token.vector *= -1;
            else if ((*iter1)->token.type == EPMatrix3x3Val) (*iter1)->token.matrix *= -1;
            else
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number, vector or matrix\n";
                return ExpressionParserValue(EPError);
            }
        }

        else if ((*iter1)->token.type == EPPostfixOperator)
        {
            delete *iter1;
            iter1 = parser->m_ExpressionParserTokenList.erase(iter1);
            iter1--;
            if ((*iter1)->token.type != EPMatrix3x3Val)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting 3x3 matrix\n";
                return ExpressionParserValue(EPError);
            }
            (*iter1)->token.matrix = (*iter1)->token.matrix.Transpose();
        }
    }

    // handle ^
    for (iter1 = parser->m_ExpressionParserTokenList.begin(); iter1 != parser->m_ExpressionParserTokenList.end(); iter1++)
    {
        if ((*iter1)->token.type == EPOperator)
        {
            if ((*iter1)->token.op == '^')
            {
                iter2 = iter1;
                iter1--;
                if ((*iter1)->token.type != EPNumber)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                    return ExpressionParserValue(EPError);
                }
                iter2++;
                if ((*iter2)->token.type != EPNumber)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                    return ExpressionParserValue(EPError);
                }
                (*iter1)->token.value = pow((*iter1)->token.value, (*iter2)->token.value);
                iter2--;
                delete *iter2;
                iter2 = parser->m_ExpressionParserTokenList.erase(iter2);
                delete *iter2;
                iter2 = parser->m_ExpressionParserTokenList.erase(iter2);
            }
        }
    }

    // handle * /
    for (iter1 = parser->m_ExpressionParserTokenList.begin(); iter1 != parser->m_ExpressionParserTokenList.end(); iter1++)
    {
        if ((*iter1)->token.type == EPOperator)
        {
            if ((*iter1)->token.op == '/')
            {
                iter2 = iter1;
                iter1--;
                if ((*iter1)->token.type != EPNumber)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                    return ExpressionParserValue(EPError);
                }
                iter2++;
                if ((*iter2)->token.type != EPNumber)
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                    return ExpressionParserValue(EPError);
                }
                (*iter1)->token.value = (*iter1)->token.value / (*iter2)->token.value;
                iter2--;
                delete *iter2;
                iter2 = parser->m_ExpressionParserTokenList.erase(iter2);
                delete *iter2;
                iter2 = parser->m_ExpressionParserTokenList.erase(iter2);
            }
            else if ((*iter1)->token.op == '*')
            {
                iter2 = iter1;
                iter1--;
                if ((*iter1)->token.type == EPNumber)
                {
                    iter2++;
                    if ((*iter2)->token.type == EPNumber)
                    {
                        (*iter1)->token.value = (*iter1)->token.value * (*iter2)->token.value;
                    }
                    else if ((*iter2)->token.type == EPMatrix3x3Val)
                    {
                        (*iter1)->token.matrix = (*iter2)->token.matrix * (*iter1)->token.value;
                        (*iter1)->token.type = EPMatrix3x3Val;
                    }
                    else if ((*iter2)->token.type == EPVectorVal)
                    {
                        (*iter1)->token.vector = (*iter2)->token.vector * (*iter1)->token.value;
                        (*iter1)->token.type = EPVectorVal;
                    }
                    else
                    {
                        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                        return ExpressionParserValue(EPError);
                    }
                }
                else if ((*iter1)->token.type == EPMatrix3x3Val)
                {
                    iter2++;
                    if ((*iter2)->token.type == EPNumber)
                    {
                        (*iter1)->token.matrix *= (*iter2)->token.value;
                    }
                    else if ((*iter2)->token.type == EPMatrix3x3Val)
                    {
                        (*iter1)->token.matrix = (*iter1)->token.matrix * (*iter2)->token.matrix;
                    }
                    else if ((*iter2)->token.type == EPVectorVal)
                    {
                        (*iter1)->token.vector = (*iter1)->token.matrix * (*iter2)->token.vector;
                        (*iter1)->token.type = EPVectorVal;
                    }
                    else
                    {
                        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                        return ExpressionParserValue(EPError);
                    }
                }
                else if ((*iter1)->token.type == EPVectorVal)
                {
                    iter2++;
                    if ((*iter2)->token.type == EPNumber)
                    {
                        (*iter1)->token.vector *= (*iter2)->token.value;
                    }
                    else if ((*iter2)->token.type == EPMatrix3x3Val)
                    {
                        (*iter1)->token.vector = (*iter1)->token.vector * (*iter2)->token.matrix;
                    }
                    else
                    {
                        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                        return ExpressionParserValue(EPError);
                    }
                }
                iter2--;
                delete *iter2;
                iter2 = parser->m_ExpressionParserTokenList.erase(iter2);
                delete *iter2;
                iter2 = parser->m_ExpressionParserTokenList.erase(iter2);

            }
        }
    }

    // and the rest just in the order it occurs
    ExpressionParserValue lhs, rhs;
    lhs = EvaluateToken(parser->m_ExpressionParserTokenList[0]);
    for (i = 1; i < parser->m_ExpressionParserTokenList.size() - 1; i += 2)
    {
        if (parser->m_ExpressionParserTokenList[i]->token.type != EPOperator)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting operator\n";
            return ExpressionParserValue(EPError);
        }
        rhs = EvaluateToken(parser->m_ExpressionParserTokenList[i + 1]);
        lhs = EvaluateOperator(lhs, rhs, parser->m_ExpressionParserTokenList[i]->token.op);
    }

    return lhs;
}

// Evaluate a token
ExpressionParserValue ExpressionParser::EvaluateToken(ExpressionParserToken *token)
{
    switch(token->token.type)
    {
        case EPNumber:
        case EPMatrix3x3Val:
        case EPVectorVal:
            break;

        case EPParser:
            token->token = EvaluateParser(token->parserList[0]);
            break;

        case EPFunction:
            token->token = EvaluateFunction(token);
            break;

        case EPVariable:
            token->token = EvaluateVariable(token);
            break;

        case EPVector:
            token->token = EvaluateVector(token);
            break;

        case EPMatrix3x3:
            token->token = EvaluateMatrix3x3(token);
            break;

        case EPNumberList:
            token->token = EvaluateNumberList(token);
            break;

        default:
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid token type\n" << token->token.type << "\n";
            break;
    }
    return token->token;
}

// Evaluate a variable
ExpressionParserValue ExpressionParser::EvaluateVariable(ExpressionParserToken *token)
{
    token->token = EvaluateGlobalVariable(token);
    if (token->token.type != EPError) return token->token;

    ExpressionFolder *p;
    ExpressionRef *r;
    r = dynamic_cast<ExpressionRef *>(m_Folder->Search(token->name, true));
    if (r)
        p = dynamic_cast<ExpressionFolder *>(r->GetLinked(true));
    else
        p = dynamic_cast<ExpressionFolder *>(m_Folder->Search(token->name, true));
    if (p == 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unrecognised name\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }
    if (p->GetStatus() != 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unresolved name\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }

    if (dynamic_cast<ExpressionVar *>(p))
    {
        token->token.type = EPNumber;
        token->token.value = dynamic_cast<ExpressionVar *>(p)->GetValue();
        if (gDebug == ParserDebug) std::cerr << __FILE__ << " " << __LINE__ << " " << token->name << " " << token->token.value << "\n";
        return token->token;
    }
    if (dynamic_cast<ExpressionVec *>(p))
    {
        token->token.type = EPVectorVal;
        token->token.vector = dynamic_cast<ExpressionVec *>(p)->GetVector();
        return token->token;
    }
    if (dynamic_cast<ExpressionMat *>(p))
    {
        token->token.type = EPMatrix3x3Val;
        token->token.matrix = dynamic_cast<ExpressionMat *>(p)->GetMatrix3x3();
        return token->token;
    }

    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Wrong folder type\n" << token->name << "\n";
    return ExpressionParserValue(EPError);
}

// Evaluate a function
ExpressionParserValue ExpressionParser::EvaluateFunction(ExpressionParserToken *token)
{
    token->token = EvaluateGlobalFunction(token);
    if (token->token.type != EPError) return token->token;

    ExpressionFunTransform3D *p;
    ExpressionRef *r;
    r = dynamic_cast<ExpressionRef *>(m_Folder->Search(token->name, true));
    if (r)
        p = dynamic_cast<ExpressionFunTransform3D *>(r->GetLinked(true));
    else
        p = dynamic_cast<ExpressionFunTransform3D *>(m_Folder->Search(token->name, true));
    if (p == 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Function not found\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }

    token->token = EvaluateParser(token->parserList[0]);

    if (token->token.type != EPVectorVal)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Wrong argument type\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }

    if (p->Calculate(&token->token.vector))
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Function not complete\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }
    return token->token;
}

// Evaluate a vector
ExpressionParserValue ExpressionParser::EvaluateVector(ExpressionParserToken *token)
{
    ExpressionVec *p;
    ExpressionRef *r;
    r = dynamic_cast<ExpressionRef *>(m_Folder->Search(token->name, true));
    if (r)
        p = dynamic_cast<ExpressionVec *>(r->GetLinked(true));
    else
        p = dynamic_cast<ExpressionVec *>(m_Folder->Search(token->name, true));

    if (p == 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unrecognised name\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }
    if (p->GetStatus() != 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unresolved name\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }

    int offset = EvaluateParser(token->parserList[0]).value;
    std::vector<double> *data = p->GetData();
    if (offset < 0 || offset >= (int)data->size())
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Vector out of range\n" << token->name << "\n" << offset << "\n";
        return ExpressionParserValue(EPError);
    }
    token->token.value = (*data)[offset];

    token->token.type = EPNumber;
    return token->token;
}

// Evaluate a Matrix3x3
ExpressionParserValue ExpressionParser::EvaluateMatrix3x3(ExpressionParserToken *token)
{
    ExpressionMat *p;
    ExpressionRef *r;
    r = dynamic_cast<ExpressionRef *>(m_Folder->Search(token->name, true));
    if (r)
        p = dynamic_cast<ExpressionMat *>(r->GetLinked(true));
    else
        p = dynamic_cast<ExpressionMat *>(m_Folder->Search(token->name, true));

    if (p == 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unrecognised name\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }
    if (p->GetStatus() != 0)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unresolved name\n" << token->name << "\n";
        return ExpressionParserValue(EPError);
    }

    int row = int(EvaluateParser(token->parserList[0]).value);
    int col = int(EvaluateParser(token->parserList[1]).value);
    std::vector<double> *data = p->GetData();
    if (row < 0 || row >= p->GetRows() || col < 0 || col >= p->GetCols())
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Matrix3x3 out of range\n" << token->name << "\n" << row << " " << col << "\n";
        return ExpressionParserValue(EPError);
    }
    token->token.value = (*data)[col + row * p->GetCols()];

    token->token.type = EPNumber;
    return token->token;
}

// Evaluate a number list
ExpressionParserValue ExpressionParser::EvaluateNumberList(ExpressionParserToken *token)
{
    if (token->parserList.size() != 3)
    {
        std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Wrong list size\n" << token->name << " " << token->parserList.size() << "\n";
        return ExpressionParserValue(EPError);
    }
    ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
    ExpressionParserValue epv1 = EvaluateParser(token->parserList[1]);
    ExpressionParserValue epv2 = EvaluateParser(token->parserList[2]);

    if (epv0.type == EPNumber && epv1.type == EPNumber && epv2.type == EPNumber)
    {
        token->token.type = EPVectorVal;
        token->token.vector.x = epv0.value;
        token->token.vector.y = epv1.value;
        token->token.vector.z = epv2.value;
        return token->token;
    }

    if (epv0.type == EPVectorVal && epv1.type == EPVectorVal && epv2.type == EPVectorVal)
    {
        token->token.type = EPMatrix3x3Val;
        token->token.matrix.e11 = epv0.vector.x;
        token->token.matrix.e12 = epv0.vector.y;
        token->token.matrix.e13 = epv0.vector.z;
        token->token.matrix.e21 = epv1.vector.x;
        token->token.matrix.e22 = epv1.vector.y;
        token->token.matrix.e23 = epv1.vector.z;
        token->token.matrix.e31 = epv2.vector.x;
        token->token.matrix.e32 = epv2.vector.y;
        token->token.matrix.e33 = epv2.vector.z;
        return token->token;
    }

    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Wrong types name\n" << token->name << "\n";
    return ExpressionParserValue(EPError);
}


// Evaluate an operator
ExpressionParserValue ExpressionParser::EvaluateOperator(ExpressionParserValue lhs, ExpressionParserValue rhs, int op)
{
    switch(op)
    {
        case '^':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            lhs.value = pow(lhs.value, rhs.value);
            return lhs;
        }

        case '+':
        {
            if (lhs.type == EPNumber)
            {
                if (rhs.type == EPNumber)
                {
                    lhs.value = lhs.value + rhs.value;
                }
                else if (rhs.type == EPMatrix3x3Val)
                {
                    pgd::Matrix3x3 t = pgd::Matrix3x3(lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value);
                    lhs.matrix = t + rhs.matrix;
                    lhs.type = EPMatrix3x3Val;
                }
                else if (rhs.type == EPVectorVal)
                {
                    pgd::Vector t = pgd::Vector(lhs.value, lhs.value, lhs.value);
                    lhs.vector = t + rhs.vector;
                    lhs.type = EPVectorVal;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            else if (lhs.type == EPMatrix3x3Val)
            {
                if (rhs.type == EPNumber)
                {
                    pgd::Matrix3x3 t = pgd::Matrix3x3(rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value);
                    lhs.matrix += t;
                }
                else if (rhs.type == EPMatrix3x3Val)
                {
                    lhs.matrix = lhs.matrix + rhs.matrix;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            else if (lhs.type == EPVectorVal)
            {
                if (rhs.type == EPNumber)
                {
                    pgd::Vector t = pgd::Vector(rhs.value, rhs.value, rhs.value);
                    lhs.vector += t;
                }
                else if (rhs.type == EPVectorVal)
                {
                    lhs.vector += rhs.vector;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            return lhs;
        }

        case '-':
        {
            if (lhs.type == EPNumber)
            {
                if (rhs.type == EPNumber)
                {
                    lhs.value = lhs.value - rhs.value;
                }
                else if (rhs.type == EPMatrix3x3Val)
                {
                    pgd::Matrix3x3 t = pgd::Matrix3x3(lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value, lhs.value);
                    lhs.matrix = t - rhs.matrix;
                    lhs.type = EPMatrix3x3Val;
                }
                else if (rhs.type == EPVectorVal)
                {
                    pgd::Vector t = pgd::Vector(lhs.value, lhs.value, lhs.value);
                    lhs.vector = t - rhs.vector;
                    lhs.type = EPVectorVal;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            else if (lhs.type == EPMatrix3x3Val)
            {
                if (rhs.type == EPNumber)
                {
                    pgd::Matrix3x3 t = pgd::Matrix3x3(rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value, rhs.value);
                    lhs.matrix -= t;
                }
                else if (rhs.type == EPMatrix3x3Val)
                {
                    lhs.matrix = lhs.matrix - rhs.matrix;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            else if (lhs.type == EPVectorVal)
            {
                if (rhs.type == EPNumber)
                {
                    pgd::Vector t = pgd::Vector(rhs.value, rhs.value, rhs.value);
                    lhs.vector -= t;
                }
                else if (rhs.type == EPVectorVal)
                {
                    lhs.vector -= rhs.vector;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            return lhs;
        }

        case '*':
            if (lhs.type == EPNumber)
            {
                if (rhs.type == EPNumber)
                {
                    lhs.value = lhs.value * rhs.value;
                }
                else if (rhs.type == EPMatrix3x3Val)
                {
                    lhs.matrix = rhs.matrix * lhs.value;
                    lhs.type = EPMatrix3x3Val;
                }
                else if (rhs.type == EPVectorVal)
                {
                    lhs.vector = rhs.vector * lhs.value;
                    lhs.type = EPVectorVal;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            else if (lhs.type == EPMatrix3x3Val)
            {
                if (rhs.type == EPNumber)
                {
                    lhs.matrix *= rhs.value;
                }
                else if (rhs.type == EPMatrix3x3Val)
                {
                    lhs.matrix = lhs.matrix * rhs.matrix;
                }
                else if (rhs.type == EPVectorVal)
                {
                    lhs.vector = lhs.matrix * rhs.vector;
                    lhs.type = EPVectorVal;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }
            else if (lhs.type == EPVectorVal)
            {
                if (rhs.type == EPNumber)
                {
                    lhs.matrix *= rhs.value;
                }
                else
                {
                    std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Invalid type combination\n";
                    return ExpressionParserValue(EPError);
                }
            }

        case '/':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            lhs.value = lhs.value / rhs.value;
            return lhs;
        }

        case '&':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value != 0 && rhs.value != 0) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case '|':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value != 0 || rhs.value != 0) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case '!':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value != rhs.value) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case '=':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value == rhs.value) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case '<':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value < rhs.value) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case '>':
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value > rhs.value) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case ('<' + 256):
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value <= rhs.value) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        case ('>' + 256):
        {
            if (lhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (rhs.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Expecting number\n";
                return ExpressionParserValue(EPError);
            }
            if (lhs.value >= rhs.value) lhs.value = 1;
            else lhs.value = 0;
            return lhs;
        }

        default:
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Unrecognised operator\n";
            return ExpressionParserValue(EPError);
    }
}

bool ExpressionParser::VariableStartTest(const char *p)
{
    if (VariableStartCharacter(*p))
    {
        if (*p == '.')
        {
            return VariableStartCharacter(*(p + 1)); // can't have a number as second character
        }
        return true;
    }
    return false;
}

bool ExpressionParser::VariableStartCharacter(int c)
{
    if (c >= 'a' && c <= 'z') return true;
    if (c >= 'A' && c <= 'Z') return true;
    if (c == '.') return true;
    if (c == '_') return true;
    return false;
}

bool ExpressionParser::VariableMiddleCharacter(int c)
{
    if (c >= 'a' && c <= 'z') return true;
    if (c >= 'A' && c <= 'Z') return true;
    if (c >= '0' && c <= '9') return true;
    if (c == '.') return true;
    if (c == '_') return true;
    return false;
}

// add extra global functions as required
ExpressionParserValue ExpressionParser::EvaluateGlobalFunction(ExpressionParserToken *token)
{
    if (token->name == "RotMat")
    {
        if (token->parserList.size() == 3)
        {
            ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
            ExpressionParserValue epv1 = EvaluateParser(token->parserList[1]);
            ExpressionParserValue epv2 = EvaluateParser(token->parserList[2]);
            if (epv0.type != EPVectorVal || epv1.type != EPVectorVal || epv2.type != EPVectorVal)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Invalid argument types\n" << token->name << " " << token->parserList.size() << "\n";
                return ExpressionParserValue(EPError);
            }
            pgd::Vector p1 = epv0.vector;
            pgd::Vector p2 = epv1.vector;
            pgd::Vector p3 = epv2.vector;
            pgd::Vector xaxis = p2 - p1; xaxis.Normalize();
            pgd::Vector cp = ClosestPoint(p3, p1, xaxis);
            pgd::Vector yaxis = p3 - cp; yaxis.Normalize();
            pgd::Vector zaxis = xaxis ^ yaxis; zaxis.Normalize();
            pgd::Matrix3x3 matrix(xaxis.x, yaxis.x, zaxis.x, xaxis.y, yaxis.y, zaxis.y, xaxis.z, yaxis.z, zaxis.z);
            token->token.type = EPMatrix3x3Val;
            token->token.matrix = matrix;
            return token->token;
        }
        else if (token->parserList.size() == 2)
        {
            ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
            ExpressionParserValue epv1 = EvaluateParser(token->parserList[1]);
            if (epv0.type != EPNumber || epv1.type != EPNumber)
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Invalid argument types\n" << token->name << " " << token->parserList.size() << "\n";
                return ExpressionParserValue(EPError);
            }
            double rot = epv0.value;
            int axisSwitch = int(epv1.value + 0.5);
            pgd::Vector axis;
            if (axisSwitch == 0) axis = pgd::Vector(1, 0, 0);
            else if (axisSwitch == 1) axis = pgd::Vector(0, 1, 0);
            else if (axisSwitch == 2) axis = pgd::Vector(0, 0, 1);
            else
            {
                std::cerr << __FILE__ << " " << __LINE__ << " " << "Invalid axis\n" << token->name << " " << token->parserList.size() << "\n";
                return ExpressionParserValue(EPError);
            }
            pgd::Quaternion q = pgd::MakeQFromAxis(axis.x, axis.y, axis.z, rot);
            token->token.type = EPMatrix3x3Val;
            token->token.matrix = pgd::MakeMFromQ(q);
            return token->token;
        }
        else
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Wrong list size\n" << token->name << " " << token->parserList.size() << "\n";
            return ExpressionParserValue(EPError);
        }
    }
    else if (token->name == "sin")
    {
        ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
        if (epv0.type != EPNumber)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Number expected\n" << token->name << "\n";
            return ExpressionParserValue(EPError);
        }
        token->token.type = EPNumber;
        token->token.value = sin(epv0.value);
        return token->token;
    }
    else if (token->name == "cos")
    {
        ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
        if (epv0.type != EPNumber)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Number expected\n" << token->name << "\n";
            return ExpressionParserValue(EPError);
        }
        token->token.type = EPNumber;
        token->token.value = cos(epv0.value);
        return token->token;
    }
    else if (token->name == "if")
    {
        ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
        if (epv0.type != EPNumber)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Number expected\n" << token->name << "\n";
            return ExpressionParserValue(EPError);
        }
        if (epv0.value != 0) // true condition
        {
            return EvaluateParser(token->parserList[1]);
        }
        else
        {
            return EvaluateParser(token->parserList[2]);
        }
    }
    else if (token->name == "v" || token->name == "g") // this is how the genome values are passed in
    {
        ExpressionParserValue epv0 = EvaluateParser(token->parserList[0]);
        if (epv0.type != EPNumber)
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: Number expected\n" << token->name << "\n";
            return ExpressionParserValue(EPError);
        }
        int index = (int)(epv0.value + 0.5); // set index to nearest integer
        if (index < 0 || index >= (int)m_v.size())
        {
            std::cerr << __FILE__ << " " << __LINE__ << " " << "Parse Error: v index out of range\n" << token->name << "\n";
            return ExpressionParserValue(EPError);
        }
        token->token.type = EPNumber;
        token->token.value = m_v[index];
        return token->token;
    }
    else
    {
        token->token.type = EPError;
    }
    return token->token;
}

// add extra global variables as required
ExpressionParserValue ExpressionParser::EvaluateGlobalVariable(ExpressionParserToken *token)
{
    if (token->name == "pi")
    {
        token->token.value = M_PI;
        token->token.type = EPNumber;
        return token->token;
    }
    else if (token->name == "x")
    {
        token->token.value = 0;
        token->token.type = EPNumber;
        return token->token;
    }
    else if (token->name == "y")
    {
        token->token.value = 1;
        token->token.type = EPNumber;
        return token->token;
    }
    else if (token->name == "z")
    {
        token->token.value = 2;
        token->token.type = EPNumber;
        return token->token;
    }
    else
    {
        token->token.type = EPError;
    }
    return token->token;
}


void ExpressionParser::Tokenizer(const char *constbuf, std::vector<std::string> &tokens, const char *stopList)
{
    char *ptr = (char *)malloc(strlen(constbuf) + 1);
    strcpy(ptr, constbuf);
    char *qp;
    char byte;
    char oneChar[2] = {0, 0};
    if (stopList == 0) stopList = "{};,=:&|!()+-/*[]'<>^";

    while (*ptr)
    {
        // find non-whitespace
        if (*ptr < 33)
        {
            ptr++;
            continue;
        }

        // is it in stoplist
        if (strchr(stopList, *ptr))
        {
            oneChar[0] = *ptr;
            tokens.push_back(oneChar);
            ptr++;
            continue;
        }

        // is it a double quote?
        if (*ptr == '"')
        {
            ptr++;
            qp = strchr(ptr, '"');
            if (qp)
            {
                *qp = 0;
                tokens.push_back(ptr);
                *qp = '"';
                ptr = qp + 1;
                continue;
            }
        }

        qp = ptr;
        while (*qp >= 33 && strchr(stopList, *qp) == 0 && *qp != '"')
        {
            qp++;
        }
        byte = *qp;
        *qp = 0;
        tokens.push_back(ptr);
        if (byte == 0) break;
        *qp = byte;
        ptr = qp;
    }
}

void ExpressionParser::SetVData(unsigned int size, double *data)
{
    if (size == m_v.size())
    {
        for (unsigned int i = 0; i < size; i++)
            m_v[i] = data[i];
    }
    else
    {
        m_v.clear();
        for (unsigned int i = 0; i < size; i++)
            m_v.push_back(data[i]);
    }
}

