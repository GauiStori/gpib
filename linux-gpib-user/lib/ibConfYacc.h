/* A Bison parser, made by GNU Bison 3.8.2.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2021 Free Software Foundation,
   Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

#ifndef YY_GPIB_YY_IBCONFYACC_H_INCLUDED
# define YY_GPIB_YY_IBCONFYACC_H_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int gpib_yydebug;
#endif

/* Token kinds.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    YYEMPTY = -2,
    YYEOF = 0,                     /* "end of file"  */
    YYerror = 256,                 /* error  */
    YYUNDEF = 257,                 /* "invalid token"  */
    T_INTERFACE = 258,             /* T_INTERFACE  */
    T_DEVICE = 259,                /* T_DEVICE  */
    T_NAME = 260,                  /* T_NAME  */
    T_MINOR = 261,                 /* T_MINOR  */
    T_BASE = 262,                  /* T_BASE  */
    T_IRQ = 263,                   /* T_IRQ  */
    T_DMA = 264,                   /* T_DMA  */
    T_PAD = 265,                   /* T_PAD  */
    T_SAD = 266,                   /* T_SAD  */
    T_TIMO = 267,                  /* T_TIMO  */
    T_EOSBYTE = 268,               /* T_EOSBYTE  */
    T_BOARD_TYPE = 269,            /* T_BOARD_TYPE  */
    T_PCI_BUS = 270,               /* T_PCI_BUS  */
    T_PCI_SLOT = 271,              /* T_PCI_SLOT  */
    T_REOS = 272,                  /* T_REOS  */
    T_BIN = 273,                   /* T_BIN  */
    T_INIT_S = 274,                /* T_INIT_S  */
    T_DCL = 275,                   /* T_DCL  */
    T_XEOS = 276,                  /* T_XEOS  */
    T_EOT = 277,                   /* T_EOT  */
    T_MASTER = 278,                /* T_MASTER  */
    T_LLO = 279,                   /* T_LLO  */
    T_EXCL = 280,                  /* T_EXCL  */
    T_INIT_F = 281,                /* T_INIT_F  */
    T_AUTOPOLL = 282,              /* T_AUTOPOLL  */
    T_SYSFS_DEVICE_PATH = 283,     /* T_SYSFS_DEVICE_PATH  */
    T_SERIAL_NUMBER = 284,         /* T_SERIAL_NUMBER  */
    T_NUMBER = 285,                /* T_NUMBER  */
    T_STRING = 286,                /* T_STRING  */
    T_BOOL = 287,                  /* T_BOOL  */
    T_TIVAL = 288                  /* T_TIVAL  */
  };
  typedef enum yytokentype yytoken_kind_t;
#endif
/* Token kinds.  */
#define YYEMPTY -2
#define YYEOF 0
#define YYerror 256
#define YYUNDEF 257
#define T_INTERFACE 258
#define T_DEVICE 259
#define T_NAME 260
#define T_MINOR 261
#define T_BASE 262
#define T_IRQ 263
#define T_DMA 264
#define T_PAD 265
#define T_SAD 266
#define T_TIMO 267
#define T_EOSBYTE 268
#define T_BOARD_TYPE 269
#define T_PCI_BUS 270
#define T_PCI_SLOT 271
#define T_REOS 272
#define T_BIN 273
#define T_INIT_S 274
#define T_DCL 275
#define T_XEOS 276
#define T_EOT 277
#define T_MASTER 278
#define T_LLO 279
#define T_EXCL 280
#define T_INIT_F 281
#define T_AUTOPOLL 282
#define T_SYSFS_DEVICE_PATH 283
#define T_SERIAL_NUMBER 284
#define T_NUMBER 285
#define T_STRING 286
#define T_BOOL 287
#define T_TIVAL 288

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
union YYSTYPE
{
#line 121 "ibConfYacc.y"

int  ival;
char *sval;
char bval;
char cval;

#line 140 "./ibConfYacc.h"

};
typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif

/* Location type.  */
#if ! defined YYLTYPE && ! defined YYLTYPE_IS_DECLARED
typedef struct YYLTYPE YYLTYPE;
struct YYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define YYLTYPE_IS_DECLARED 1
# define YYLTYPE_IS_TRIVIAL 1
#endif




int gpib_yyparse (void *parse_arg, void* yyscanner);


#endif /* !YY_GPIB_YY_IBCONFYACC_H_INCLUDED  */
