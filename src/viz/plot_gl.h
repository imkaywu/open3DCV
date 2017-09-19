#ifndef plot_h_
#define plot_h_

#include <GL/glew.h> // include GLEW and new version of GL on Windows
#include <GLFW/glfw3.h> // GLFW helper library
#include "image/image.h"

namespace open3DCV
{
    inline void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
    {
        std::cout << key << std::endl;
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GL_TRUE);
        }
    }
    
    inline int gl_init()
    {
        if (!glfwInit()) {
            fprintf(stderr, "Error: could not start GLFW3\n");
            return 1;
        }
        
        // uncomment these lines if on macOS
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        
        GLFWwindow* window = glfwCreateWindow(640, 480, "Hello Triangle", NULL, NULL);
        if (!window) {
            std::cout << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return 1;
        }
        glfwMakeContextCurrent(window);
        
        // set the required keyboard callback functions
        glfwSetKeyCallback(window, key_callback);
        
        // set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
        glewExperimental = GL_TRUE;
        // initialize GLEW to setup the OpenGL function pointers
        if (glewInit() != GLEW_OK) {
            std::cout << "Failed to initialize GLEW" << std::endl;
            return -1;
        }
        
        // get version info
        const GLubyte* renderer = glGetString(GL_RENDERER); // get renderer string
        const GLubyte* version = glGetString(GL_VERSION); // version as a string
        printf("Renderer: %s\n", renderer);
        printf("OpenGL version supported %s\n", version);
        
        // tell GL to only draw onto a pixel if the shape is closer to the viewer
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        
        /* OTHER STUFF GOES HERE NEXT */
        float points[] = {0.0f,  0.5f, 0.0f,
            0.5f, -0.5f, 0.0f,
            -0.5f, -0.5f, 0.0f};
        GLuint vbo = 0;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, 9 * sizeof(float), points, GL_STATIC_DRAW);
        
        GLuint vao = 0;
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
        
        const char* vertex_shader =
        "#version 400\n"
        "in vec3 vp;"
        "void main() {"
        "    gl_Position = vec4(vp, 1.0);"
        "}";
        
        const char* fragment_shader =
        "#version 400\n"
        "out vec4 frag_colour;"
        "void main() {"
        "    frag_colour = vec4(0.5, 0.0, 0.5, 1.0);"
        "}";
        
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vertex_shader, NULL);
        glCompileShader(vs);
        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fragment_shader, NULL);
        glCompileShader(fs);
        
        GLuint shader_program = glCreateProgram();
        glAttachShader(shader_program, fs);
        glAttachShader(shader_program, vs);
        glLinkProgram(shader_program);
        
        
        while (!glfwWindowShouldClose(window)) {
            // wipe the drawing surface clear
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glUseProgram(shader_program);
            glBindVertexArray(vao);
            // draw points 0-3 from the currently bound VAO with current in-use shader
            glDrawArrays(GL_TRIANGLES, 0, 3);
            // update other events like input handling
            glfwPollEvents();
            // put the stuff we've been drawing onto the display
            glfwSwapBuffers(window);
        }
        
        
        // close GL context and any other GLFW resources
        glfwTerminate();
        return 0;
    }

}

#endif // end of namespace open3DCV
