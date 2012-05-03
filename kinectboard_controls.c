// vim:ts=4:sw=4:expandtab

#include "kinectboard_controls.h"
#include <malloc.h>

static uint32_t CONTROL_ID = 0;

/* ******************************************************************* */
/* KB Utility functions */
int __kb_mouse_over(int mouse_x, int mouse_y, SDL_Rect* target) {
     if (( mouse_x > target->x )
                  && ( mouse_x < target->x + target->w )
                  && ( mouse_y > target->y )
                  && ( mouse_y < target->y + target->h )) {
              return 1;
          }
    return 0;
}

SDL_Surface* kb_surface_fill_color(SDL_Rect* box, SDL_Color* color) {
    Uint32 rmask, gmask, bmask; //, amask;
      #if SDL_BYTEORDER == SDL_BIG_ENDIAN
         rmask = 0xff000000;
         gmask = 0x00ff0000;
         bmask = 0x0000ff00;
       //  amask = 0x000000ff;
      #else
         rmask = 0x000000ff;
         gmask = 0x0000ff00;
         bmask = 0x00ff0000;
       //  amask = 0xff000000;
      #endif
      
    SDL_Surface* surface = SDL_CreateRGBSurface(SDL_SWSURFACE, box->w, box->h, 24/*32*/, rmask, gmask, bmask, /*amask*/0);
    SDL_LockSurface(surface);
    SDL_FillRect(surface, box, SDL_MapRGB(surface->format, color->r, color->g, color->b));
    SDL_UnlockSurface(surface);
    
    return surface;
}

int __compare_btn(kb_button* lhs, kb_button* rhs) {
    return (lhs != 0 && rhs != 0) ? (rhs->id == lhs->id ? 0 : 1) : -1; 
}

int __compare_label(kb_label* lhs, kb_label* rhs) {
    return (lhs != 0 && rhs != 0) ? (rhs->id == lhs->id ? 0 : 1) : -1;
}

int __compare_slider(kb_slider* lhs, kb_slider* rhs) {
    return (lhs != 0 && rhs != 0) ? (rhs->id == lhs->id ? 0 : 1) : -1; 
}

void __kb_unregister_control(kb_controls* list, void* control) {
    if(!list) { return; }
    if(list->root->type == KB_BUTTON) {
        if(__compare_btn((kb_button*)list->root, (kb_button*)control) == 0) {
            list->root = list->root->next;
        } 
    }
    
     if(list->root->type == KB_SLIDER) {
        if(__compare_slider((kb_slider*)list->root, (kb_slider*)control) == 0) {
            list->root = list->root->next;
        } 
    }
    
    if(list->root->type == KB_LABEL) {
        if(__compare_label((kb_label*)list->root, (kb_label*)control) == 0) {
            list->root = list->root->next;
        }
    }

    kb_controls_node* ptr = list->root;	
	while(ptr != 0) {
        if(ptr->type == KB_BUTTON && ptr->data != 0
	    && __compare_btn((kb_button*)ptr->data, (kb_button*)control) == 0) {
            ptr->data = ptr->next;
        }
	    if(ptr->type == KB_SLIDER && ptr->data != 0
	    && __compare_slider((kb_slider*)ptr->data, (kb_slider*)control) == 0) {
            ptr->data = ptr->next;
        }
        if(ptr->type == KB_LABEL && ptr->data != 0
	    && __compare_label((kb_label*)ptr->data, (kb_label*)control) == 0) {
            ptr->data = ptr->next;
        }
    }
}

/* ******************************************************************* */
/* KB Controls Logic */

kb_controls* kb_controls_create() 
{
    kb_controls* list = (kb_controls*)malloc(sizeof(kb_controls));
    list->root = 0;
    list->size = 0;
    return list; 
}

void kb_controls_destroy(kb_controls* ptr)
{
    free(ptr);
    ptr = 0;
}

void kb_controls_add_control(kb_controls* list, kb_type type, void* ptr)
{
    kb_controls_node* node = (kb_controls_node*)malloc(sizeof(kb_controls_node));
    node->parent = 0;
    node->next = 0;
    node->data = ptr;
    node->type = type;
    
    if(!list) { 
	    free(node); 
	    return; 
    }
    
    if(!list->root && list->size == 0)  {
	    list->size++;
	    list->root = node;
	    return;
    }
    
    kb_controls_node* next = list->root; 
    
    while(next->next != 0) {
        next = next->next;
    }
    
    
    next->next = node;
    node->parent = next;
    list->size++;
}

void kb_process_mouse_motion(kb_controls* list, uint8_t button, int x, int y, int x_rel, int y_rel)
{    
    if(!list) { return; } 

	kb_controls_node* ptr = list->root;
		
	while(ptr != 0) {
        if(ptr->data != 0) {
            if(ptr->type == KB_BUTTON) {
                kb_button* btn = (kb_button*)ptr->data;
                btn->state = __kb_mouse_over(x, y, &btn->box); 
            }
	    if(ptr->type == KB_SLIDER) {
                kb_slider* slider = (kb_slider*)ptr->data;
		if(button == 1) {
		    int min_x = slider->box.x - slider->knob_box.w/2;
		    int max_x = slider->box.x+slider->box.w -slider->knob_box.w/2;
		    
		    if(slider->state > 0 && x > 0) {
			if(__kb_mouse_over(x, y, &slider->box)) {
			    slider->state = 2;
			}
			slider->knob_box.x = x < min_x ? min_x : (x > max_x ? max_x : x-(slider->knob_box.w/2));
			float perc = (0.f+slider->knob_box.x-slider->box.x+(slider->knob_box.w/2)) / (0.f+slider->box.w);
			slider->callback(perc < 0.f ? 0.f : (perc > 100.f ? 100.f : perc));
		    }
		} else {
		    slider->state = __kb_mouse_over(x, y, &slider->knob_box); 
		}
            }
        }
        ptr = ptr->next;
	}
}

bool kb_process_input(kb_controls* list, uint8_t button, int x, int y)
{    
    if(!list) { 
	return false; 
    } 

    kb_controls_node* ptr = list->root;	
	while(ptr != 0) {
        if(ptr->data != 0) {
            if(ptr->type == KB_BUTTON) {
                kb_button* btn = (kb_button*)ptr->data;
		if(__kb_mouse_over(x, y, &btn->box) > 0) {
                    btn->callback(0);
		    return true;
                }
            } 
	    
	    if(ptr->type == KB_SLIDER) {
		kb_slider* slider = (kb_slider*)ptr->data;
                if(__kb_mouse_over(x, y, &slider->box) > 0) {
		    slider->knob_box.x = x - (slider->knob_box.w / 2);
		    float perc = (0.f+slider->knob_box.x-slider->box.x+(slider->knob_box.w/2)) / (0.f+slider->box.w);
		    slider->callback(perc < 0.f ? 0.f : (perc > 100.f ? 100.f : perc));
		    return true;
		}
	    } 
	}
	ptr = ptr->next;
    }
    
    return false;
}
 
void kb_controls_render(kb_controls* list, SDL_Surface* screen) 
{
	if(!list) { return; } 

	kb_controls_node* ptr = list->root;
	while(ptr != 0) {
        if(ptr->data != 0) {
            if(ptr->type == KB_BUTTON) {
                kb_button* btn = (kb_button*)ptr->data;
                SDL_BlitSurface(btn->state > 0 ?  btn->btn_hover : btn->btn_norm, NULL, screen, &btn->box);
            } else if(ptr->type == KB_SLIDER) {
                kb_slider* slider = (kb_slider*)ptr->data;	
		
                SDL_BlitSurface(slider->slider_pane, NULL, screen, &slider->pane_box);
		
                SDL_BlitSurface(slider->state > 0 ?  slider->slider_knob_hover : slider->slider_knob_norm, NULL, screen, &slider->knob_box);
            } else if (ptr->type == KB_LABEL) {
				kb_label* label = (kb_label*)ptr->data;
                SDL_BlitSurface(label->text, NULL, screen, &label->textLocation);
			}
        }
		
        ptr = ptr->next;
	}
}

/* ******************************************************************* */
/* KB Button */
kb_button* kb_button_create(kb_controls* list, int width, int height, int xpos, int ypos, kb_button_cb button_pressed_callback, const char* buttonText, TTF_Font* font)
{
    kb_button* btn = (kb_button*)malloc(sizeof(kb_button));
    
    btn->callback = button_pressed_callback;

    btn->box.w = width;
    btn->box.h = height;
    
    // Apply offsets later
    btn->box.x = 0;
    btn->box.y = 0;
    
    // Add some color
    SDL_Color color_normal = {255,220,100};
    SDL_Color color_hover = {255,240,100};

    btn->btn_norm  = kb_surface_fill_color(&btn->box, &color_normal);
    btn->btn_hover = kb_surface_fill_color(&btn->box, &color_hover);
    
    // apply offset
    btn->box.x = xpos;
    btn->box.y = ypos;

    btn->state = 0;
    btn->id = ++CONTROL_ID;
    
    kb_controls_add_control(list, KB_BUTTON, btn);
    SDL_Color foregroundColor = { 0, 0, 0};
    btn->text = TTF_RenderText_Solid(font, buttonText, foregroundColor);
    //TextLocation muss noch angepasst werden.
    SDL_Rect textLocation = { 0, 0, 0, 0 };
    SDL_BlitSurface(btn->text, NULL, btn->btn_norm, &textLocation);
    SDL_BlitSurface(btn->text, NULL, btn->btn_hover, &textLocation);
	
    return btn;
}

kb_label* kb_label_create(kb_controls* list, int xpos, int ypos, const char* labelText, TTF_Font* font) {
	kb_label* label = (kb_label*)malloc(sizeof(kb_label));
	label->textLocation = (SDL_Rect){ xpos, ypos, 0, 0 };
	label->color = (SDL_Color){255,255,255};
	label->text = TTF_RenderText_Solid(font, labelText, label->color);
	label->id = ++CONTROL_ID;
	label->font = font;
	kb_controls_add_control(list, KB_LABEL, label);

	return label;
}

void kb_label_changeText(kb_label* label, const char* newText) {
	SDL_FreeSurface(label->text);
	label->text = TTF_RenderText_Solid(label->font, newText, label->color);
}

void kb_button_destroy(kb_controls* list, kb_button* btn) 
{
	__kb_unregister_control(list, btn);
	SDL_FreeSurface(btn->btn_norm);
	SDL_FreeSurface(btn->btn_hover);
	SDL_FreeSurface(btn->text);
	
	free(btn);
	btn = 0;
}

void kb_label_destroy(kb_controls* list, kb_label* label)
{
	__kb_unregister_control(list, label);
	SDL_FreeSurface(label->text);
	free(label);
	label = 0;
}
/* ******************************************************************* */
/* KB Slider */
kb_slider* kb_slider_create(kb_controls* list, int width, int height, int xpos, int ypos, kb_slider_cb slider_moved_callback, float initial_value)
{
    if(initial_value < 0.f || initial_value > 100.f) {
	printf("ERROR creating slider: Initial value must be between 0.0 and 100.0; value was %f", initial_value);
	fflush(stdout);
	return 0;
    }
    
    kb_slider* slider = (kb_slider*)malloc(sizeof(kb_slider));
    
    slider->callback = slider_moved_callback;
    
    slider->box.w = width;
    slider->box.h = height;
    
    // Apply offsets later
    slider->box.x = 0;
    slider->box.y = 0;
   
    
    // Add some color
    SDL_Color color_pane = {120,50,10};
    SDL_Color color_normal = {255,20,100};
    SDL_Color color_hover = {255,240,100};
   

    // Create pane
    slider->pane_box.w = width;
    slider->pane_box.h = 4;
    slider->pane_box.x = 0;
    slider->pane_box.y = 0;
    slider->slider_pane = kb_surface_fill_color(&slider->pane_box, &color_pane);
    slider->pane_box.x = xpos;
    slider->pane_box.y = ypos+(slider->box.h/4);
    
    // Create Knob
    slider->knob_box.w = 10;
    slider->knob_box.h = height;
    slider->knob_box.x = 0;
    slider->knob_box.y = 0;
    slider->slider_knob_norm  = kb_surface_fill_color(&slider->knob_box, &color_normal);
    slider->slider_knob_hover = kb_surface_fill_color(&slider->knob_box, &color_hover);
    
    slider->knob_box.x = xpos+(slider->box.w * initial_value/100.f)-(slider->knob_box.w/2);
    slider->knob_box.y = ypos-(slider->box.h/4);
    
    // apply offset
    slider->box.x = xpos;
    slider->box.y = ypos;
    
    slider->state = 0;
    slider->id = ++CONTROL_ID;
    
    
    kb_controls_add_control(list, KB_SLIDER, slider);

    return slider;
}

void kb_slider_destroy(kb_controls* list, kb_slider* slider)
{
	__kb_unregister_control(list, slider);
	SDL_FreeSurface(slider->slider_knob_norm);
	SDL_FreeSurface(slider->slider_knob_hover);
	SDL_FreeSurface(slider->slider_pane);
	
	free(slider);
	slider = 0;
}
