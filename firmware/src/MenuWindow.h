#ifndef _MENUWINDOW_H_
#define _MENUWINDOW_H_

#include <Arduino.h>

#define MAX_STR_SIZE 64
#define MAX_NO_OF_OPTS 10

class MenuWindow{
    private:
        char name[MAX_STR_SIZE];
        char options[MAX_NO_OF_OPTS][MAX_STR_SIZE];
        void (*opt_callbacks[MAX_NO_OF_OPTS])(void);
        int opt_return_values[MAX_NO_OF_OPTS];
        int name_len;
        int no_of_options;

    public:

        MenuWindow();
        MenuWindow(const char *title);
        MenuWindow(const char *title, const char **options, int no_of_options);
        bool set_name(const char *name);
        bool add_option(const char *option_name);
        bool add_option(const char *option_name, int return_value);
        bool add_option(const char *option_name, int return_value, void (*callback)(void));
        bool is_option_valid(int option);
        bool option_has_callback(int option);
        int get_option_value(int option);
        void exec_callback(int option);
        void print();
};

#endif