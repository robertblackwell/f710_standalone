#include <stdio.h>
#include <stdlib.h>
#include <cctype>
#include "argv.h"
#include <cstring>


static bool tokenize_line(const char* line, Argv& argv)
{
    if(strlen(line) > ARGV_BUFFER_MAX) {
        printf("tokenize error line is too long");
        return false;
    }
    strcpy(&(argv.m_line_buffer[0]), line);
    strcpy(&(argv.m_unmodified_line_buffer[0]), line);

    argv.token_count = 0;
    char* buf = argv.m_line_buffer;
    char* tk = strtok(buf, " ");
    
    while(tk != nullptr) {
        if(strcmp(tk, " ") != 0) {
            argv.token_positions[argv.token_count] = tk;
            argv.token_lengths[argv.token_count] = strlen(tk);
            argv.token_count++;
            if(argv.token_count >= ARGV_TOKENS_MAX) {
                printf("tokenize too many tokens");
                return false;
            }
            argv.token_positions[argv.token_count] = nullptr;
            argv.token_lengths[argv.token_count] = 0;

        }
        tk = strtok(nullptr, " ");
    } 
    return true;
}
Argv::Argv()
{
    token_count = 0;
    m_line_buffer[0] = '\0';
    m_unmodified_line_buffer[0] = '\0';
    std::memset(token_lengths, 0, sizeof(token_lengths));
    std::memset(token_positions, 0, sizeof(token_positions));
}
Argv::Argv(const char* line) 
{
    token_count = 0;
    m_line_buffer[0] = '\0';
    m_unmodified_line_buffer[0] = '\0';
    std::memset(token_lengths, 0, sizeof(token_lengths));
    std::memset(token_positions, 0, sizeof(token_positions));
    tokenize_line(line, *this);
}
bool Argv::tokenize(const char* line)
{
    return tokenize_line(line, *this);
}
void Argv::dump(const char* msg)
{
    printf("Dump Argv msg:%s  tokens: %p token_count: %d\n", msg, (this), this->token_count);
    for(int i = 0; i < this->token_count; i++) {
        printf("i: %d  token: %p[%s] len: %d\n", i, this->token_positions[i], this->token_positions[i], this->token_lengths[i]);
    }
}

void Argv::copyTo(Argv& other)
{
    other.token_count = this->token_count;
    char* src_origin = &(this->m_line_buffer[0]);
    char* dest_origin = &(other.m_line_buffer[0]);
    for(int i = 0; i < this->token_count; i++) {
        char* p_src = this->token_positions[i];
        char* p_dest = dest_origin + (p_src - src_origin);
        other.token_positions[i] = p_dest;
        other.token_lengths[i] = this->token_lengths[i];
        strcpy(other.token_positions[i], this->token_positions[i]);
    }
}

const char* Argv::token_at(const int i) const
{
    return this->token_positions[i];
}

/*
Functions for parsing argument tokens
*/
static bool isposnum(const char* p, int& v) {
    int tmp = 0;
    const char* pp = p;
    while(*pp != static_cast<char>(0)) {
        if(!isdigit(*pp)) {
            return false;
        }
        tmp = tmp * 10 + (int)(*pp - '0');
        pp++;
    }
    v = tmp;
    return true;
}

static bool isnegnum(const char* p, int& v) {
    int tmp;
    if(*p == '-') {
        if(isposnum(&p[1], tmp)) {
            v = -1 * tmp;
            return true;
        }
    }
    return false;
}

static bool isdouble(const char* p, double& fv) {
    
    const char* q = p;
    if(*q == '\0') {
        return false;
    }
    if(*q == '-') {
        q++;
    }
    if(*q == '\0') {
        return false;
    }
    while(isdigit(*q)) {
        q++;
    }
    if(*q == '\0') {
        fv = atof(p);
        return true;
    } else if(*q == '.') {
        q++;
    } else {
        return  false;
    }
    while(isdigit(*q)) {
        q++;
    }
    if(*q != '\0') {
        return false;
    }
    fv = atof(p);
    return true;
}
bool argparse_posint(const Argv& args, const int at, int& return_value)
{
    return isposnum(args.token_at(at), return_value);
}
bool argparse_negint(const Argv& args, const int at, int& return_value)
{
    return isnegnum(args.token_at(at), return_value);
}
bool argparse_double(const Argv& args, const int at, double& dv)
{
    return isdouble(args.token_at(at), dv);
}
