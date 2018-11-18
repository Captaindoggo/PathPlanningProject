#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char *argv[])
{
    stringstream convert(argv[1]);
    int n;
    convert >> n;
    for (int i = 0; i != n; ++i) {
        cout << "pathplanning" <<'\n';
    }
    return 0;
}
