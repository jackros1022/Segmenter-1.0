/* Shader
 *
 * Copyright (C) 2005, Maurizio Monge <maurizio.monge@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _TG_SHADER_H__
#define _TG_SHADER_H__



#include "headers.h"
#include "tgMathlib.h"

namespace TomGine{

/** @author Maurizio Monge, Thomas Mörwald
 *	@brief Managing shader for GLSL (fragment-, vertex- and header files for shader).
 */
class tgShader{
private:
    GLhandleARB fragment;
    GLhandleARB vertex;
    GLhandleARB program;
public:
    /** @brief Creating the shader programs.
     *  @param	vertex_file	Path to vertex shader file.
     *  @param fragment_file	Path to fragment shader file.
     *  @param vertex_header_file	Path to header file, which is included before vertex shader string.
     *  @param fragment_header_file	Path to header file, which is included before fragment shader string. */
    tgShader(	const char *vertex_file = NULL,
    			const char *fragment_file = NULL,
    			const char *vertex_header_file = NULL,
    			const char *fragment_header_file = NULL);
    /** @brief Destroys shader programs. */
    ~tgShader();

    /** @brief Binds/Activates shader program for usage. */
    void bind();
    /** @brief Unbinds/Disables shader program. */
    void unbind();
    /** @brief Prints variables of shader program to console. */
    void dumpVars();
    /** @brief	Prints info log of shader program to console. */
    void printInfoLog(GLhandleARB obj, const char *msg, ...);
    /** @brief	Checks if shader program is initialised. (true=initialised, false=not valid). */
    bool getStatus();

    /** @brief	Returns the location of an attribute variable of the bound shader. */
    GLuint getAttribLoc(const char*);
    /** @brief	Returns the location of an uniform variable of the bound shader. */
    GLint  getUniformLoc(const char*);
    /** @brief Specify the value of a 'uniform int' variable for the current program object */
    void setUniform(const char*,int);
    /** @brief Specify the value of a 'uniform unsigned' variable for the current program object */
    void setUniform(const char*,unsigned);
    /** @brief Specify the value of a 'uniform int[]' variable for the current program object */
    void setUniform(const char*,int,const int*);
    /** @brief Specify the value of a 'uniform float' variable for the current program object */
    void setUniform(const char*,float);
    /** @brief Specify the value of a 'uniform float[]' variable for the current program object */
    void setUniform(const char*,int,const float*);
    /** @brief Specify the value of a 'uniform vec2' variable for the current program object */
    void setUniform(const char*,vec2);
    /** @brief Specify the value of a 'uniform vec2[]' variable for the current program object */
    void setUniform(const char* var,int n,vec2* f);
    /** @brief Specify the value of a 'uniform vec3' variable for the current program object */
    void setUniform(const char*,vec3);
    /** @brief Specify the value of a 'uniform vec3[]' variable for the current program object */
    void setUniform(const char* var,int n,vec3* f);
    /** @brief Specify the value of a 'uniform vec4' variable for the current program object */
    void setUniform(const char*,vec4);
    /** @brief Specify the value of a 'uniform vec4[]' variable for the current program object */
    void setUniform(const char* var,int n,vec4* f);
    /** @brief Specify the value of a 'uniform mat3' variable for the current program object */
    void setUniform(const char*,mat3,bool transpose=false);
    /** @brief Specify the value of a 'uniform mat3[]' variable for the current program object */
    void setUniform(const char* var,int n,mat3* f,bool transpose);
    /** @brief Specify the value of a 'uniform mat4' variable for the current program object */
    void setUniform(const char*,mat4,bool transpose=false);
    /** @brief Specify the value of a 'uniform mat4[]' variable for the current program object */
    void setUniform(const char* var,int n,mat4* f,bool transpose);

};


/* allocate (and free) aligned memory, align must be power of 2 */
void *malloc_align(size_t size, int align);
void free_align(void *ptr);

/* read entierely a file, returning a 0 terminated string */
char *read_text_file(const char* file);

/* offset of a membet in a structure */
#undef OFFSETOF
#define OFFSETOF(TYPE, MEMBER) (((size_t)&((TYPE *)1)->MEMBER) - 1)

/* some templates */
template<typename T>
T Max(T a,T b)
{
    return a>b?a:b;
}

template<typename T>
T Min(T a,T b)
{
    return a<b?a:b;
}

template<typename T>
T Abs(T a)
{
    return a>0?a:-a;
}

template<typename T>
T Square(T a)
{
    return a*a;
}

template<typename T>
T Sign(T a)
{
    return a==0?0:a>0?1:-1;
}

} //namespace TomGine

#endif //__SHADER_H__
