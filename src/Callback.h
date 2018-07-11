//
// Created by smntin on 06.07.18.
//

#ifndef PROJECT_CALLBACK_H
#define PROJECT_CALLBACK_H

class Callback{
private:
    void (*function)(char*,void*);
    void  *parameters[4];

public:
    Callback(){
        function=[](char*,void*){
        };
    }

    template<class T>
    void operator=(T func){
        // Вот так мы убедимся, что  sizeof(T) <= sizeof(parameters)
        // Если это не выполняется, то будет compile-time ошибка, т.к.
        // нельзя указывать отрицательный размер массива
        sizeof(int[ sizeof(parameters)-sizeof(T) ]);

        // Сохраняем указатель на функцию, которая вызывает переданную функцию func
        function=[](char* arg,void *param){
            (*(T*)param)(arg);
        };

        // Копируем значение в переменной func в parameters
        memcpy(parameters,&func,sizeof(T));
    }

    void operator()(char* d){
        // Вызываем функцию по указателю function, передав ещё и parameters
        function(d,parameters);
    }
};

#endif //PROJECT_CALLBACK_H
