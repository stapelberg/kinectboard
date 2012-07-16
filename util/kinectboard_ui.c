// vim:ts=4:sw=4:expandtab

#include "kinectboard_ui.h"
#include <malloc.h>
#include <pthread.h>

#if 0
static uint32_t CONTROL_ID = 0;
static pthread_mutex_t rendermutex = PTHREAD_MUTEX_INITIALIZER;
#endif

#include "Awesomium/awesomium_capi.h"

#include <GL/glew.h>
#include <GL/gl.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include <unistd.h>
#include <limits.h>

// two kinect images (full resolution) next to each other
#define SCREEN_WIDTH (640 * 2)
// kinect image height (480) + space for controls
#define SCREEN_HEIGHT 300
#define SCREEN_OFFSET 480
#define SCREEN_DEPTH 4
#define URL "ui.html" 
#define JSOBJECT "Kinectboard"

static awe_webview* webView = 0;

// Global JS Object identifier
static awe_string* obj_str = 0;
static GLuint textureID;
static GLuint textureID_front;
static GLuint textureID_back;
static GLuint bufferID;
static unsigned char* buffer = 0;

/* ************************************************************************* */
#pragma pack(push)
typedef struct
{ 
  uint8_t funct;
  kb_ui_js_value_callback val_callback;
  kb_ui_js_void_callback callback;

} cb_funct;
#pragma pack(pop)
/* ************************************************************************* */
#pragma pack(push)
typedef struct
{
    const char* funct_name;
    uint8_t funct_type;
    void* callback;
    void* next;
} cb_node;
#pragma pack(pop)

/* ************************************************************************* */
#pragma pack(push)
typedef struct 
{
    cb_node* head;
    uint32_t size;
} cb_table;
#pragma pack(pop)

/* ************************************************************************* */
static cb_table callback_table;

/* ************************************************************************* */
void kb_ui_inject_mouse_button(uint8_t button, bool down) {
    awe_mousebutton btn;

    if (button == SDL_BUTTON_LEFT) {
        btn = AWE_MB_LEFT;
    }
    else if (button == SDL_BUTTON_MIDDLE) {
        btn = AWE_MB_MIDDLE;
    }
    else if (button == SDL_BUTTON_RIGHT) {
        btn = AWE_MB_RIGHT;
    }

    if(down)
        awe_webview_inject_mouse_down(webView, btn);
    else
        awe_webview_inject_mouse_up(webView, btn);
}

/* ************************************************************************* */
void kb_ui_inject_mouse(Uint16 x, Uint16 y) {
    Uint16 yy = y;
    if(yy > SCREEN_OFFSET) {
        yy -= SCREEN_OFFSET;
    } else return;
    awe_webview_inject_mouse_move(webView,x,yy);
}

/* ************************************************************************* */
cb_node* ui_create_node(const char* js_function_match, int funct, void* callback) {
    cb_node* node= (cb_node*)malloc(sizeof(cb_node));
    node->funct_name= js_function_match;
    node->funct_type = funct;
    node->callback = callback;
    node->next = 0;
    return node;

}
/* ************************************************************************* */
void ui_create_object_cb(const char* method) {
    awe_string* e_str = awe_string_create_from_ascii(method, strlen(method));
    awe_webview_set_object_callback(webView,obj_str,e_str);
    awe_string_destroy(e_str);
}
/* ************************************************************************* */
void do_register_callback(const char* js_function_match, int funct, void* callback) {
    if(callback_table.size == 0) {
        ui_create_object_cb(js_function_match);
        callback_table.head = ui_create_node(js_function_match,funct,callback);
        callback_table.size++;
        return;
    }

    cb_node* next = callback_table.head; 
    
    while(next->next != 0) {
        next = next->next;
    }

    ui_create_object_cb(js_function_match);
    next->next = ui_create_node(js_function_match,funct, callback);
    callback_table.size++;
}
/* ************************************************************************* */
void kb_ui_register_value_callback(const char* js_function_match, kb_ui_js_value_callback callback) {
    do_register_callback(js_function_match, 1, callback);
}

/* ************************************************************************* */
void kb_ui_register_void_callback(const char* js_function_match, kb_ui_js_void_callback callback) {
    do_register_callback(js_function_match, 0, callback);
}
/* ************************************************************************* */
void ui_javascript_callback_funct(awe_webview* caller, const awe_string* object_name,
                                             const awe_string* callback_name,
                                             const awe_jsarray* arguments) {
    printf("DEBUG: Received Callback: ");
    fflush(stdout);
    size_t size =awe_string_to_utf8(callback_name, NULL, 0);

    char* buf = (char*)malloc(size);
    awe_string_to_utf8(callback_name, buf, size);

    printf("%s\n", buf);
    fflush(stdout);

    cb_node* next = callback_table.head; 

    if(!next) {
        free(buf);
        return;
    }
    do {
       if(strcmp(next->funct_name, buf) == 0) {
            // Void Callbacks
            if(next->funct_type == 0) {
                ((kb_ui_js_void_callback)next->callback)();
                return;
            }

            // Callbacks with values
            if(awe_jsarray_get_size(arguments) < 1) {
                printf("Error: kb_ui_js_value_callback expects the JS function to pass arguments! \n");
                return;
            }
            const awe_jsvalue* val = awe_jsarray_get_element(arguments, 0);
            ((kb_ui_js_value_callback)next->callback)((float)awe_jsvalue_to_double(val));
            
        }
        next = next->next;
    } while(next != 0);

    free(buf);
}

/* ************************************************************************* */
void kb_ui_call_javascript(const char* function, char* argv) {
    awe_string* funct_str = awe_string_create_from_ascii(function,strlen(function));
    awe_string* args_str = awe_string_create_from_ascii(argv,strlen(argv));
    awe_jsvalue* val = awe_jsvalue_create_string_value(args_str);

    const awe_jsvalue* _arr[] = { val };
    awe_jsarray* args_arr = awe_jsarray_create(_arr, 1);

    awe_webview_call_javascript_function(
        webView,
        awe_string_empty(),
        funct_str,
        args_arr,
        awe_string_empty());

    awe_jsvalue_destroy(val);
    awe_jsarray_destroy(args_arr);
    awe_string_destroy(funct_str);
    awe_string_destroy(args_str);
}

/* ************************************************************************* */
void kb_ui_init() {
    obj_str = awe_string_create_from_ascii(JSOBJECT, strlen(JSOBJECT));

    callback_table.size = 0;
    callback_table.head = 0;

    // Create the WebCore with the default options
    awe_webcore_initialize_default();
    char cwd[PATH_MAX];
    getcwd(cwd, PATH_MAX);
    awe_string* cwd_str = awe_string_create_from_ascii(cwd,
                               strlen(cwd));
    awe_webcore_set_base_directory(cwd_str);

    awe_string* url_str = awe_string_create_from_ascii(URL,
                               strlen(URL));
    // Create a new WebView to load our page
    webView = awe_webcore_create_webview(SCREEN_WIDTH,
                                  SCREEN_HEIGHT,
                                  false);

    // Load the URL into our WebView instance
    awe_webview_load_file(webView,url_str,awe_string_empty());

    // Destroy our URL string
    awe_string_destroy(url_str);

    awe_webview_focus(webView);

    // Wait for WebView to finish loading the page
    while(awe_webview_is_loading_page(webView))
    awe_webcore_update();

    awe_webview_create_object(webView,obj_str);
    awe_webview_set_callback_js_callback(webView, ui_javascript_callback_funct);

    const awe_renderbuffer* rend_buffer = awe_webview_render(webView);

    if(!rend_buffer)
    {
        printf("Invalid RenderBuffer (Awesomium)\n");
        exit(0);
    }
    glEnable(GL_TEXTURE_2D);
    glGenBuffers(1, &bufferID);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
    glBufferData(GL_PIXEL_UNPACK_BUFFER, SCREEN_WIDTH*SCREEN_HEIGHT*SCREEN_DEPTH, awe_renderbuffer_get_buffer(rend_buffer), GL_DYNAMIC_DRAW);
    glGenTextures(1, &textureID_front);
    glBindTexture(GL_TEXTURE_2D, textureID_front);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SCREEN_WIDTH, SCREEN_HEIGHT, 0, GL_BGRA, GL_UNSIGNED_BYTE, 0);
}

/* ************************************************************************* */
void kb_ui_render() {
    glDisable( GL_DEPTH_TEST ) ;

    glViewport( 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT );
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, SCREEN_WIDTH, 0, SCREEN_HEIGHT);
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();  
    glScalef(1,1,1);

    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
    glBindTexture(GL_TEXTURE_2D, textureID_front);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

    glBegin(GL_QUADS);
        glTexCoord2f(0,0);
        glVertex3f(0, (GLfloat)SCREEN_HEIGHT, 0.0f);
        glTexCoord2f(1,0);
        glVertex3f((GLfloat)SCREEN_WIDTH, (GLfloat)SCREEN_HEIGHT, 0.0f);
        glTexCoord2f(1,1);
        glVertex3f((GLfloat)SCREEN_WIDTH, 0, 0.0f);
        glTexCoord2f(0,1);
        glVertex3f(0, 0, 0.0f);
    glEnd();

}

/* ************************************************************************* */
void kb_ui_update() {
    awe_webcore_update();
   if(awe_webview_is_dirty(webView)) {

        const awe_renderbuffer* rend_buffer = awe_webview_render(webView);
        if(rend_buffer)
        {
            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
            void * p = glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
            memcpy(p, awe_renderbuffer_get_buffer(rend_buffer), SCREEN_WIDTH*SCREEN_DEPTH*SCREEN_HEIGHT);
            glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);

            glBindBuffer(GL_PIXEL_UNPACK_BUFFER, bufferID);
            glBindTexture(GL_TEXTURE_2D, textureID_front);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
        }
    }

}
/* ************************************************************************* */
void kb_ui_destroy() {
    awe_webview_destroy(webView);
    awe_webcore_shutdown(); 
    glDeleteTextures( 1, &textureID);
}