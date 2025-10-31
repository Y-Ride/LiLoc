#include <iostream>

using namespace std;


class ParamTest {
    public:
    bool alive = true;
    int val = 5;

    void setAlive(bool input) {alive = input;}
    void setVal(int input) {val = input;}
    
};

class Class1 : public ParamTest {
    public:
    Class1() {
        print_params();
    }

    void print_params()
    {
        cout << "Class1 Params" << endl;
        cout << "\tAlive: " << this->alive << endl;
        cout << "\tVal:   " << this->val << endl;
    }
};

class Class2 : public ParamTest {
    public:
    Class2() {
        print_params();
    }

    void print_params()
    {
        cout << "Class2 Params" << endl;
        cout << "\tAlive: " << this->alive << endl;
        cout << "\tVal:   " << this->val << endl;
    }
};

int main() {

    ParamTest pt;

    pt.setAlive(false);
    pt.setVal(10);

    Class1 class1;
    Class2 class2;


    class1.print_params();
    class2.print_params();

    return 0;
}
