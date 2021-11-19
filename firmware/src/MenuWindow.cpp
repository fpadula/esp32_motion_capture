#include "MenuWindow.h"

bool MenuWindow::set_name(const char *name){
    strncpy(this->name, name, MAX_STR_SIZE);
    this->name_len = strlen(this->name);
    return true;
}

bool MenuWindow::add_option(const char *option_name){
    strncpy(this->options[this->no_of_options], option_name, MAX_STR_SIZE);
    this->opt_callbacks[this->no_of_options] = NULL;
    this->opt_return_values[this->no_of_options] = 0;
    this->no_of_options++;
    return true;
}

bool MenuWindow::add_option(const char *option_name, int return_value){
    strncpy(this->options[this->no_of_options], option_name, MAX_STR_SIZE);
    this->opt_callbacks[this->no_of_options] = NULL;
    this->opt_return_values[this->no_of_options] = return_value;
    this->no_of_options++;
    return true;
}

bool MenuWindow::add_option(const char *option_name, int return_value, void (*callback)(void)){
    strncpy(this->options[this->no_of_options], option_name, MAX_STR_SIZE);
    this->opt_callbacks[this->no_of_options] = callback;
    this->opt_return_values[this->no_of_options] = return_value;
    this->no_of_options++;
    return true;
}

void MenuWindow::exec_callback(int option){
    this->opt_callbacks[option]();
}

int MenuWindow::get_option_value(int option){
    return this->opt_return_values[option];
}

MenuWindow::MenuWindow(){
    this->no_of_options = 0;
    this->set_name("");
    // this->add_option("Back");
}

MenuWindow::MenuWindow(const char *title){
    this->no_of_options = 0;
    this->set_name(title);
    // this->add_option("Back");
}

MenuWindow::MenuWindow(const char *title, const char **options, int no_of_options){
    int i;

    this->no_of_options = 0;
    this->set_name(title);
    for(i = 0; i < no_of_options; i++)
        this->add_option(options[i]);
}

void MenuWindow::print(){
    int i, header_len;
    char buffer[128];

    sprintf(buffer, "############### %s ###############\n", this->name);
    header_len = strlen(buffer) - 1;
    Serial.print(buffer);
    for(i = 0; i < this->no_of_options; i++){
        sprintf(buffer, "\t[%d] %s\n", i, options[i]);
        Serial.print(buffer);
    }
    for(i = 0; i < header_len; i++)
        sprintf(buffer+i, "#");
    Serial.println(buffer);
}

bool MenuWindow::is_option_valid(int option){
    return (option >= 0) && (option < this->no_of_options);
}

bool MenuWindow::option_has_callback(int option){
    return this->opt_callbacks[option] != NULL;
}