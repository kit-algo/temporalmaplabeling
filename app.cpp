#include "app.h"

#include <cstdlib>
#include <iostream>
#include <cxxabi.h>

#include "map/projection.h"


/* BLACK MAGIC from https://stackoverflow.com/questions/4885334/c-finding-the-type-of-a-caught-default-exception/24997351#24997351 */
using namespace __cxxabiv1;

std::string util_demangle(std::string to_demangle)
{
    int status = 0;
    char * buff = __cxxabiv1::__cxa_demangle(to_demangle.c_str(), NULL, NULL, &status);
    std::string demangled = buff;
    std::free(buff);
    return demangled;
}

App::App(int &argc, char *argv[]):
    QApplication(argc, argv)
{

}

bool App::notify(QObject *receiver, QEvent *e)
{
    try {
       return QApplication::notify(receiver, e);
    } catch (const char *ex) {
        std::cout << "\nCaught a string...\n" << ex << std::endl;
        // Magic rethrow is happening here.
        //std::cout << "\nCaught exception type: '" << util_demangle(__cxa_current_exception_type()->name()) << "'" << std::endl;
       //throw;
    }

    return false;
}

