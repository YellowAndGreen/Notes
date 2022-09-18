# Trick

## --a和a--

a++ 和++a 都是将a 加1，但是a++ 返回值为a，而++a 返回值为a+1。如果只是希望增加a 的值，而不需要返回值，则推荐使用++a，其运行速度会略快一些。

**因为先加1再输出的方式省去了将其放入寄存器的时间**



## main命令行参数

> int argc:          英文名为arguments count(参数计数)
> char** argv:    英文名为arguments value/vector(参数值)
> argv[0] 指向程序运行时的全路径名
> argv[i] 指向程序在DOS命令中执行程序名后的第i个字符串
> argv[argc]数组越界.

```c++
int main(int argc,char** argv)
{
....
return 0; // 可以不写这句话，在main中若不写return则默认有return 0;
}
```

## 暂停程序

```c++
// 程序暂停一下，按任意键继续
system("pause");
```





# 变量

## 初始化

```c++
int main()
{
// 三种方式
int x = 123;
int y{ 123 };
int z = { 123 };
}
```

## 基本类型

```c++
char c = 'a';
int y = -256;
double d = 3.14;
std::string s = "123";
//常量，定义时必须先初始化
const int x =10
// 常量，在编译时初始化（也就是最开始）
constexpr int z = 20
constexpr int z1 = z
constexpr int z2 = y // 错误，编译时还不知道y的值
```

### 自动类型推断

```c++
auto c = 'a'
auto d = 1
auto& y = c
```

### implicit转换

```c++
int main()
{
    char mychar = 64;
    int myint = 123;
    double mydouble = 456.789;
    bool myboolean = true;
    myint = mychar;
    mydouble = myint;
    mychar = myboolean;
	
    // 可以字符相加
    char c1 = 10;
    char c2 = 20;
    auto result = c1 + c2; // result is of type int
    
    // 将一个指针转换为空指针
    int x = 123;
    int* pint = &x;
    void* pvoid = pint;
    
    // 将数组转为对第一个元素的指针
    int arr[5] = { 1, 2, 3, 4, 5 };
    int* p = arr; // pointer to the first array element
    std::cout << *p;
    
    // 函数中的数组传递的是第一个元素的指针
    // prefer std:vector and std::array containers to raw arrays and pointers.
    int arr[] = {1,1,1};
    arrfunc(arr);
    /* void arrfunc(const int arr[]){
    std::cout << *arr;
	}  */
}
```

### explicit转换

1. 使用函数`static_cast<type_to_convert_to>(value_to_convert_from)`
2. 更倾向于使用直接转换

## 类型修饰符

The signed (the default if omitted) means the type can hold both positive and negative values, and unsigned means the type has unsigned representation. Other modifiers are for the size:short - type will have the width of at least 16 bits, and long - type will have the width ofat least 32 bits.

```c++
#include <iostream>
int main()
{
unsigned long int x = 4294967295;
std::cout << "The value of an unsigned long integer variable is: " << x;
}
```

## 数组

```c++
// 初始化并赋值
int arr[5] = {1,2,3,4};
std::cout << arr[1];

// 获取一个数组的长度
int a[5] ={1,2,3,4,5};
std::cout << sizeof a/sizeof a[0];
```

## 字符串

```c++
std::string s= "asddasdad";

// 获取字符子串，从2开始的3个字符
std::cout << s.substr(2,3);

// 搜索一个子串
std::string s= "asddasdad";
// size_type是这个函数类型，若没找到就返回一个npos类型
std::string::size_type found = s.find("das1");
if (found != std::string::npos)
{
    std::cout << "Substring found at position: " << found;
}
else
{
    std::cout << "The substring is not found.";
}
```

### 读取空格划分的字符串

```c++
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

 	std::fstream fs{ argv[1] };
    std::string s;
    while (fs)
    {
        std::getline(fs, s);
        std::istringstream ss(s);
        std::string point;
        std::vector<std::string> points;
        // 读取每个空格分割字符串，放入列表
        while(ss >> point) {
            points.push_back(point);
        }
        // stod将字符转为double类型
        temp_point = {stof(points[0]),stof(points[1]),stof(points[2])};
        cloud->push_back(temp_point);//从点云最后面插入一点
    }
```

### 类型转换

```c++
// 将字符串转为float类型
stof("0.2121")
// 将字符串转为double类型
stod("0.2121")
// 将字符串转换为长整型整数long int
atoll("21")
// 将字符串转换为长整数long long int
atol("21")
```



## 指针

> 指针类型<type>\*，例如int* p
>
> 使用&获取一个变量的指针
>
> 使用\*获取一个指针指向的变量

```c++
int x = 123;
int* p = &x;
std::cout << *p;

// 改变指针的对象
*p = 456
    
// 初始化空指针
char* p = nullptr
    
int x;
int * p1 = &x; // 指针可以被修改，值也可以被修改
const int * p2 = &x; // 指针可以被修改，值不可以被修改（const int）
int * const p3 = &x; // 指针不可以被修改（* const），值可以被修改
const int * const p4 = &x; // 指针不可以被修改，值也不可以被修改

// 使用逗号在一行赋值的时候，第一个以后的指针要加上*号
ListNode* slow = head , *fast = head;

// ListNode *head和ListNode* head没有区别，均表示指针 
```

### 聪明指针

> Smart pointers are pointers that own the object they point to and automatically destroy
> the object they point to and deallocate the memory once the pointers go out of scope.

#### unique指针

> A unique pointer called std::unique_ptr is a pointer that owns an object it points
> to. The pointer **can not be copied**. Unique pointer deletes the object and deallocates
> memory for it, once it goes out of scope.

使用方式：

```c++
#include <iostream>
#include <memory>    // 需要用到memory
int main()
{
std::unique_ptr<int> p(new int{ 123 });
std::cout << *p;
}
```

更好的初始化方式：

==The std::make_unique function was introduced in the C++14 standard. Make sure to compile with the` -std=c++14` flag to be able to use this function.==

```c++
#include <iostream>
#include <memory>
int main()
{
std::unique_ptr<int> p = std::make_unique<int>(123);
std::cout << *p;
}
```

使用类的属性

```c++
#include <iostream>
#include <memory>
class MyClass
{
public:
void printmessage()
{
std::cout << "Hello from a class.";
}
};
int main()
{
std::unique_ptr<MyClass> p = std::make_unique<MyClass>();
p->printmessage();
}
```



#### 共享指针

> 共享指针能够复制！
>
> 多个指针指向同一个对象，只有最后一个指针的scope结束时，才会在内存中删除对象

初始化共享指针并复制

```c++
#include <iostream>
#include <memory>
int main()
{
std::shared_ptr<int> p1 = std::make_shared<int>(123);
std::shared_ptr<int> p2 = p1;
std::shared_ptr<int> p3 = p1;
}
```



## 引用

> <type>\&表示引用类型

```c++
int x = 123;
int& p1 = x;
int p2 = x;
// 引用和原变量的指针相同，直接赋值指针不同
std::cout << &x <<"\n" << &p1 << "\n" << &p2;
```

## 生命周期

1. 自动存储期间

所有local变量均以栈存储

2. 动态存储期间

堆存储，手动指定

3. 静态存储期间

以`static`开头的变量，生命周期是整个程序运行期间

### new和delete

```c++
#include <iostream>
int main()
{
int* p = new int;
*p = 123;
std::cout << "The pointed-to value is: " << *p;
delete p;
}

// 对于数组变量
#include <iostream>
int main()
{
int* p = new int[3];
p[0] = 1;
p[1] = 2;
p[2] = 3;
std::cout << "The values are: " << p[0] << ' ' << p[1] << ' ' << p[2];
delete[] p;
}
```

### 临时变量

```c++
// name使用过后就会被删除
explicit Person(std::string name):name{std::move(name)}{}
```



## 静态变量

1. 静态变量只会进行一次初始化，后面的初始化会跳过（注意：跳过的是初始化而不是赋值！！）

```c++
#include <iostream>

void myfunction()
{
    static int x = 0; // defined only the first time, skipped every other
// time
    x++;
    std::cout << x << '\n';
}

int main()
{
    myfunction(); // x == 1
    myfunction(); // x == 2
    myfunction(); // x == 3
}
```

使用静态变量作为类的字段

```c++
#include <iostream>
class MyClass
{
public:
    static int x; // declare a static data member
};
int MyClass::x = 123; // define a static data member
int main()
{
    MyClass::x = 456; // access a static data member
    std::cout << "Static data member value is: " << MyClass::x;
}
```

静态类方法

```c++
#include <iostream>
class MyClass
{
public:
    static void myfunction(); // declare a static member function
};

// 类里声明的时候已经有static，所以这里不需要写
void MyClass::myfunction()
{
    std::cout << "Hello World from a static member function.";
}
int main()
{
    MyClass::myfunction(); // call a static member function
}
```



# 流程控制

## if

> 逻辑运算符：&&，||，!，^

> 条件表达式：`condition ? exp1 : exp2` 若条件符合则exp1，反之

```c++
std::string::size_type found = s.find("das1");
if (found != std::string::npos)
{
    std::cout << "Substring found at position: " << found;
}
else
{
    std::cout << "The substring is not found.";
}
```

## switch

```c++
#include <iostream>
int main()
{
    int x = 3;
    switch (x)
    {
        case 1:
            std::cout << "The value of x is 1.";
            break;
        case 2:
            std::cout << "The value of x is 2.";
            break;
        case 3:
            std::cout << "The value of x is 3."; 
            break;
        default:
            std::cout << "The value is none of the above.";
            break;
    }
}
```

## for

```c++
    for (int i=0;i<10;i++) {
        std::cout << i<<"\n";
    }
```



# 输入与输出

```c++
int x =0;
char y;
// 分两次输入
std::cin >>x >>y;  
std::cout << x <<"\n"<<y;

// 使用getline来获取有空格的输入（直接使用cin只会获取到空格之前的字符串）
std::string s;
std::getline(std::cin,s);
std::cout << s;
```

# 函数

1. 最好先声明，再定义后再使用

```c++
// 声明一个无返回值的函数
void myvoidfunction();

// 只声明函数的时候，可以忽略变量名
int mysum(int, int);

// 定义函数
int mysquarednumber(int x) {
return x * x;
}

// 传递变量的引用（最好避免传递引用，使用常量引用）
#include <iostream>
void myfunction(int& byreference)
{
byreference++; // we can modify the value of the argument
std::cout << "Argument passed by reference: " << byreference;
}
int main()
{
int x = 123;
myfunction(x);
}

// 使用常量引用
#include <iostream>
#include <string>
void myfunction(const std::string& byconstreference)
{
std::cout << "Arguments passed by const reference: " <<
byconstreference;
}
int main()
{
std::string s = "Hello World!";
myfunction(s);
}
```

## 函数重载

参数类型不一样

```c++
void myprint(char param);
void myprint(int param);
void myprint(double param);
```

# 面向对象

## 类

1. 不修改成员字段的类方法应该加上const前缀：`std::string getname() const { return name; }`

```c++
// 声明一个类
class MyClass{};
int main()
{
    
MyClass o;   // 新建一个类实例
}

// 也可以通过struct定义一个类
struct MyStruct
{
// everything in here
// is public by default
};

// 数据成员字段
class MyClass
{
char c;
int x;
double d;
};

//类方法
class MyClass
{
void dosomething()   // 也可以在类里声明，然后在类外面定义
{
std::cout << "Hello World from a class.";
}
};

// 在类外面定义类方法
class MyClass
{
void dosomething();
};

void MyClass::dosomething()
{
std::cout << "Hello World from a class.";
}

// new运算符返回的是一个指向所分配类型变量（对象）的指针。
// 类指针访问类属性使用箭头，使用n
MyBaseClass* o = new MyDerivedClass;
o->dowork();

// 初始化一个类
Person person{ "John Doe." };

// 类实例访问类属性
MyClass::x = 456;
```

## 权限指示符

```c++
class MyClass
{
public:
// everything in here
// has public access level
protected:
// everything in here
// has protected access level
private:
// everything in here
// has private access level
};
```

1. **private无法在类之外获取到**

```c++
#include <iostream>
class MyClass
{
private:
int x; // x now has private access
public:
void printx()
{
std::cout << "The value of x is:" << x; // x is accessible to
// printx()
}
};
int main()
{
MyClass o; // Create an object
o.x = 123; // Error, x has private access and is not accessible to
// object o
o.printx(); // printx() is accessible from object o
}
```

2. **protect**可以在子类获取到，但是在实例中（不管是自身的实例还是子类的实例）无法使用

```c++
class MyBaseClass
{
protected:
    char c;
    int x;
};


class MyDerivedClass : public MyBaseClass
{
// c and x also accessible here
};
int main()
{
    MyDerivedClass o;
    o.c = 'a'; // Error, not accessible to object
    o.x = 123; // error, not accessible to object
}
```



## 构造器

### 默认构造器

无参或者有默认参数的构造器是默认构造器

```c++
#include <iostream>
class MyClass
{
public:
MyClass()
{
std::cout << "Default constructor invoked." << '\n';
}
};
int main()
{
MyClass o; // invoke a default constructor
}
```

默认参数

```c++
#include <iostream>
class MyClass
{
public:
int x, y;
MyClass(int xx, int yy)
{
x = xx;
y = yy;
}
};
int main()
{
MyClass o{ 1, 2 }; // invoke a user-provided constructor
std::cout << "User-provided constructor invoked." << '\n';
std::cout << o.x << ' ' << o.y;
}
```

**更简洁的初始化**

```c++
#include <iostream>
class MyClass
{
public:
int x, y;
MyClass(int xx, int yy)
: x{ xx }, y{ yy } // member initializer list
{
}
};
int main()
{
MyClass o{ 1, 2 }; // invoke a user-defined constructor
std::cout << o.x << ' ' << o.y;
}
```

### 子类构造器

```c++
class Person{
    public:
        std::string name;

        void getName() const{
            std::cout << name;
        }

        explicit Person(std::string name):name{std::move(name)}{}
};

class Student : public Person{
    public:
        int semester;
    	// 使用了父类的构造器
        Student(std::string name,int semester): Person::Person(std::move(name)),semester{semester}{}
};
```



### 复制构造器

默认构造器只能进行浅拷贝

```c++
#include <iostream>
class MyClass
{
private:
int x, y;
public:
MyClass(int xx, int yy) : x{ xx }, y{ yy }
{
}
};
int main()
{
MyClass o1{ 1, 2 };
MyClass o2 = o1; // default copy constructor invoked
}
```

自定义复制构造器

```c++
#include <iostream>
class MyClass
{
private:
    int x, y;
public:
    MyClass(int xx, int yy) : x{ xx }, y{ yy }
    {
        std::cout << &x;
    }
// user defined copy constructor
    MyClass(const MyClass& rhs)
            : x{ rhs.x }, y{ rhs.y } // initialize members with other object's
// members
    {
        std::cout << "User defined copy constructor invoked.";
        std::cout << &x;
    }
};
int main()
{
    MyClass o1{ 1, 2 };
    MyClass o2 = o1; // user defined copy constructor invoked
}
```

### 移动构造器

```c++
#include <iostream>
#include <string>
class MyClass
{
private:
    double d;
    std::string s;
public:
    MyClass(double dd, std::string ss) // user-provided constructor
            : d{ dd }, s{ ss }{}

    MyClass(MyClass&& otherobject) // user-defined move constructor
            :
            d{ std::move(otherobject.d) }, s{ std::move(otherobject.s) }
    {
        std::cout << "Move constructor invoked." << '\n';
    }
    void printdata()
    {
        std::cout << "The value of doble is: " << d << ", the value of string is: " << s << '\n';
    }
};
int main()
{
    MyClass o1{ 3.14, "This was in object 1" };
    MyClass o2 = std::move(o1); // invokes the move constructor
    o2.printdata();
}
```

### 单参数构造器（防止自动类型转换）

```c++
explicit Person(const std::string& aname)
: name{ aname }
{}
```

![image-20220423164356823](img/image-20220423164356823.png)

## 继承和多态

继承

```c++
class MyBaseClass
{
};
class MyDerivedClass : public MyBaseClass
{
};
int main()
{
}
```

多态

```c++
#include <iostream>
class MyBaseClass
{
public:
    virtual void dowork()
    {
        std::cout << "Hello from a base class." << '\n';
    }
};
class MyDerivedClass : public MyBaseClass
{
public:
    void dowork() override
    {
        std::cout << "Hello from a derived class." << '\n';
    }
};
int main()
{
    MyBaseClass* o = new MyDerivedClass;
    o->dowork();
    delete o;
}
```

### 抽象类

抽象类有至少一个抽象方法（纯虚拟函数）

```c++
#include <iostream>
class MyAbstractClass
{
public:
    virtual void dowork() = 0;
};
class MyDerivedClass : public MyAbstractClass
{
public:
    void dowork()
    {
        std::cout << "Hello from a derived class." << '\n';
    }
};
int main()
{
    MyAbstractClass* o = new MyDerivedClass;
    o->dowork();
    delete o;
}
```

### 基类必须有一个虚拟destructor

This ensures the proper deallocation of objects accessed through a base class pointer via the inheritance chain:

```c++
class MyBaseClass
{
public:
    virtual void dowork() = 0;
    virtual ~MyBaseClass() {};
};
```



## 泛型

1. 定义：`template <typename T>`或`template <class T>`，两者效果一样。

```c++
#include <iostream>
template <typename T>
// 也可同时定义两个参数的泛型
// template <typename T, typename U>
void myfunction(T param)
{
    std::cout << "The value of a parameter is: " << param;
}
int main()
{
    myfunction<int>(123);
    myfunction<double>(123.456);
    myfunction<char>('A');
}
```

泛型作为类变量：

```c++
#include <iostream>
template <typename T>  // 同template <class T> 
class MyClass {
private:
    T x;
public:
    explicit MyClass(T xx)
            :x{ xx }
    {
    }
    T getvalue()
    {
        return x;
    }
};
int main()
{
    MyClass<int> o{ 123 };
    std::cout << "The value of x is: " << o.getvalue() << '\n';
    MyClass<double> o2{ 456.789 };
    std::cout << "The value of x is: " << o2.getvalue() << '\n';
}
```

## 使类实例能够像函数一样调用

```c++
#include <iostream>
class MyClass
{
public:
    // 需要覆盖这个函数
    void operator()(int x)
    {
        std::cout << "Function object with a parameter " << x << "called.";
    }
};
int main()
{
    MyClass myobject;
    myobject(123); // invoke the function object
}
```





# 代码组织

### **头文件和源文件：**

By convention, there are two kinds of files into which we can store our C++ source: ==**header files (headers) and source files**.==

> Header files usually have the .h (or .hpp) extension. Source files are files where we can store our definitions and the main program. They usually have the .cpp (or .cc) extension.

## 不同的头文件命名规则

To include user-defined header files, we use the #include statement, followed by a full header name with extension enclosed in double-quotes. Example:

```c++
#ifndef "myheader.h"     // 头文件的保卫宏
#include "myheader.h"
#include "otherheader.h"
```



==We should put the declarations and constants into header files and put definitions and executable code in source files.==





## 命名空间

```c++
#include <iostream>
namespace MyNameSpace
{
void myfunction();
}
void MyNameSpace::myfunction()
{
std::cout << "Hello World from a function inside a namespace.";
}
int main()
{
MyNameSpace::myfunction();
}
```



**嵌套命名空间**

```c++
#include <iostream>
namespace A
{
    namespace B
    {
        void myfunction();
    }
}
void A::B::myfunction()
{
    std::cout << "Hello World from a function inside a nested namespace."
              << '\n';
}
int main()
{
    A::B::myfunction();
    using namespace A::B;
    myfunction();
}
```



# 异常

```c++
#include <iostream>
int main()
{
    try
    {
        std::cout << "Let's assume some error occurred in our program." << '\n';
        std::cout << "We throw an exception of type int, for example." << '\n';
        std::cout << "This signals that something went wrong." << '\n';
        throw 123; // throw an exception if there is an error
    }
    catch (int e)
    {
        // catch and handle the exception
        std::cout << "Exception raised!." << '\n';
        std::cout << "The exception has a value of " << e << '\n';
    }
}
```



# I/O流

## 文件流

1. std::ifstream – read from a file
2. std::ofstream – write to a file
3. std::fstream – read from and write to a file



#### **读取文件**

```c++
#include <iostream>
#include <fstream>
#include <string>
int main()
{
    std::fstream fs{ "myfile.txt" };
    std::string s;
    while (fs)
    {
        std::getline(fs, s); // read each line into a string
        std::cout << s << '\n';
    }
}
```

使用>>读取一个字符

```c++
#include <iostream>
#include <fstream>
int main()
{
    std::fstream fs{ "myfile.txt" };
    char c;
    // 默认不会读取空格，使用std::noskipws读取
    while (fs >> std::noskipws >> c)
    {
        std::cout << c;
    }
}
```

#### **写入文件**

`std::ios::out`表示会覆盖写入

```c++
#include <fstream>
int main()
{
    std::fstream fs{ "myfile.txt", std::ios::out };
    fs << "First line of text." << '\n';
    fs << "Second line of text" << '\n';
    fs << "Third line of text" << '\n';
}
```

`std::ios::app`追加写入

```c++
#include <fstream>
int main()
{
    std::fstream fs{ "myfile.txt", std::ios::app };
    fs << "This is appended text" << '\n';
    fs << "This is also an appended text." << '\n';
}
```



## 字符串流

1. std::stringstream - the stream to read from a string
2. std::otringstream - the stream to write to a string
3. std::stringstream - the stream to both read from and write to a
   string



```c++
#include <iostream>
#include <sstream>
int main()
{
    // std::stringstream ss{ "Hello world." };
    std::stringstream ss;
    ss << "Hello World.";
    std::cout << ss.str(); << "xu"
}
```



## 文件路径处理

### 分离路径和文件名

```c++
std::string path = "D:\\mydoc\\VS-proj\\SMTDetector\\x64\\Release\\0001.bmp";
int index = path.find_last_of("\\");
std::cout << path.substr(index+1);
std::cout << path.substr(0,index);
```

### 文件名后缀替换

```c++
#include <string>
#include <iostream>

int main(){
    std::string path = "/home/pcl/docker_dir/example_project/data/test/1.pcd";
    path.replace(path.size()-3,path.size()-1,std::string("txt"));
    std::cout << path;
}
```

### 使用系统指令创建文件夹

```c++
// 使用系统指令创建文件夹
std::string command = "mkdir -p " + path.substr(0,index)+"/out";
system(command.c_str());
```

### 获取某目录下所有文件名

> 法1：使用系统指令，将结果保存到一个临时文件中，使用后删除即可

```c++

#include <iostream>
#include <fstream>
#include <vector>

int main(int argc, const char *argv[])
{
    // 使用系统指令将其保存
    system("touch temp.txt && ls>temp.txt");
    // 读取
    std::fstream fs{ "temp.txt" };
    // 将文件名保存到列表v中
    std::vector<std::string> v;
    std::string s;
    while (fs)
    {
        // 将每一行读入s
        std::getline(fs, s);
        // 将s放入列表中
        v.push_back(s);
    }
    // 删除临时文件
    system("rm temp.txt");
}
```



# 标准库

## 容器

![image-20220506111955978](img/image-20220506111955978.png)

![image-20220506112016233](img/image-20220506112016233.png)

### std::vector

1. vector的长度会随着删除和插入自动变化

![image-20220507092106079](img/image-20220507092106079.png)

```c++
#include <vector>
#include <iostream>

int main()
{
    std::vector<int> v = { 1, 2, 3, 4, 5 };
    // 初始化四个相同的值（0）
    std::vector<int> color(4, 0);
    // 二维数组初始化
    std::vector<std::vector<int>> graph(10,std::vector<int>());
    // 入栈操作
    v.push_back(10);
    // 两种获取元素的方法
    std::cout << v[2] << v.at(3) << "\n";
    // 获取向量的长度
    std::cout << "The vector's size is: " << v.size();
}
```

### std::array

> The std::array is a thin wrapper around a C-style array.

```c++
#include <iostream>
#include <array>
int main()
{
    std::array<int, 5> arr = { 1, 2, 3, 4, 5 };
    for (auto el : arr)
    {
        std::cout << el << '\n';
    }
}
```

### std::queue

```c++
int main() {
    std::queue<int> q;
    // 插入数据
    q.push(1);
    q.push(2);
    // 访问第一个元素
    std::cout <<q.front();
    // 访问最后一个元素
    std::cout<<q.back();
    // 判断是否为空
    std::cout<<q.empty();
    // 弹出数据
    q.pop();
    std::cout<<q.front();
}
```

### std::stack

empty() 堆栈为空则返回真

pop() 移除栈顶元素

push() 在栈顶增加元素

size() 返回栈中元素数目

top() 返回栈顶元素，不会删除栈顶元素



### std::set

与其类似还有unordered_set

> Set is a container that holds unique, sorted objects. It is a binary tree of sorted objects.

```c++
#include <iostream>
#include <set>

int main() {
    std::set<int> myset = {1, 2, 3, 4, 5};
    // 插入一个值
    myset.insert(10);
    for (auto el: myset) {
        std::cout << el << '\n';
    }
    // 判断一个值是否在set中，若不在则返回0
    std::cout << myset.count(15);
    
    // 通过迭代器初始化一个容器
    std::set<int> dict(to_delete.begin(),to_delete.end());
}
```

### std::unordered_set

> 与set不同，unordered_set是哈希表

```c++
#include <iostream>
#include <unordered_set>

int main() {
    std::unordered_set<int> myset = {1, 2, 3, 4, 5};
    std::cout << myset.count(15);
}
```

### std::unordered_map

> 同unordered_set一样，也是哈希表

Key: .first(->first)
Value: .second(->second)
插入数据：.emplace .insert
判断内部是否为空：.empty（返回bool）
返回容器大小：.size(返回unsigned integral)
返回最大的数据:max_size
交换全部key和value:first.swap(second)
全部清零：clear
查找x是否在map中：unordered_map<int, int> mp;（若存在 mp.find(x)!=mp.end()）（若存在 mp.count(x)!=0）
初始元素：.begin
结尾元素：.end
删除元素：.erase(“”)
存在某元素：count(“”)存在返回1否则返回0
查找某元素：find()

```c++
#include <iostream>
#include <unordered_map>

int main() {
    std::unordered_map<int,int> myset;
    myset.insert({1, 1});
    std::cout << myset[1];
}
```



### std::map

```c++
#include <iostream>
#include <map>
int main()
{
    std::map<int, char> mymap = { {1, 'a'}, {2, 'b'}, {3,'z'} };
    // 插入键值对
    mymap[4] = 'h';
    mymap.insert({ 20, 'c' });
    for (auto el : mymap)
    {
        std::cout << el.first << ' ' << el.second << '\n';
    }
}
```

查找一个键值对，若无结果则返回mymap.end()

```c++
#include <iostream>
#include <map>
int main()
{
    std::map<int, char> mymap = { {1, 'a'}, {2, 'b'}, {3,'z'} };
    auto it = mymap.find(2);
    if (it != mymap.end())
    {
        std::cout << "Found: " << it->first << " " << it->second << '\n';
    }
    else
    {
        std::cout << "Not found.";
    }
}
```

### std::pair

表示一对值

```c++
#include <iostream>
#include <utility>
int main()
{
std::pair<int, double> mypair = { 123, 3.14 };
auto [r, c] = mypair
std::cout << "The first element is: " << mypair.first << '\n';
std::cout << "The second element is: " << mypair.second << '\n';
}
```



## range-based循环

1. 可以循环迭代 container/range的内容：`for (some_type element_name : container_name)`

2. ==element_name 实际上是元素的复制，若想使用实际元素，可以用引用类型==：

   ```c++
   #include <iostream>
   #include <vector>
   int main()
   {
       std::vector<int> v = { 1, 2, 3, 4, 5 };
       v.push_back(10);
       for (int& el : v)
       {
           el = 2;
       }
       for (int el : v)
       {
           std::cout << el;
       }
   }
   ```

   

## 迭代器

1. 迭代器类似于容器元素的指针，通过++指向下一个元素，使用*获取指针元素

2. 最后一个元素的迭代器为.end()

   ```c++
   #include <iostream>
   #include <vector>
   int main()
   {
       std::vector<int> v = { 1, 2, 3, 4, 5 };
       for (auto it = v.begin(); it!=v.end(); it++)
       {
           std::cout << *it << '\n';
       }
   }
   ```

   

3. 使用迭代的erase()来删除一个元素

   ```c++
   #include <iostream>
   #include <vector>
   int main()
   {
       std::vector<int> v = { 1, 2, 3, 4, 5 };
       auto it = v.begin() + 3;
       v.erase(it);
       for (auto el : v)
       {
           std::cout << el << '\n';
       }
   }
   ```

   4. 通过迭代器初始化一个容器
   
      ```c++
      std::set<int> dict(to_delete.begin(),to_delete.end());
      ```
   
      

## 算法工具

### std::sort

```c++
#include <iostream>
#include <vector>
#include <algorithm>
int main()
{
    std::vector<int> v = { 1, 5, 2, 15, 3, 10 };
    // 默认从小到大
    std::sort(v.begin(), v.end());
    for (auto el : v)
    {
        std::cout << el << '\n';
    }
    // 从大到小
    std::sort(v.begin(), v.end(), std::greater<>());
    for (auto el : v)
    {
        std::cout << el << '\n';
    }
}
```

### std::find

```c++
#include <iostream>
#include <vector>
#include <algorithm>
int main()
{
    std::vector<int> v = { 1, 5, 2, 15, 3, 10 };
    auto result = std::find(v.begin(), v.end(), 5);
    if (result!=v.end())
    {
        std::cout << "Element found: " << *result;
    }
    else
    {
        std::cout << "Element not found.";
    }
}

```

### std::copy

将容器的元素拷贝到另一个元素

```c++
#include <iostream>
#include <vector>
#include <algorithm>
int main()
{
    std::vector<int> copy_from_v = { 1, 2, 3, 4, 5 };
    std::vector<int> copy_to_v(5); // reserve the space for 5 elements
    std::copy(copy_from_v.begin(), copy_from_v.end(), copy_to_v.begin());
    for (auto el : copy_to_v)
    {
        std::cout << el << '\n';
    }
}
```

### 最大最小元素

返回一个迭代器指针

```c++
#include <iostream>
#include <vector>
#include <algorithm>
int main()
{
    std::vector<int> v = { 1, 2, 3, 4, 5 };
    auto it = std::max_element(std::begin(v), std::end(v));
    std::cout << "The max element in the vector is: " << *it;
}
```

### 求和std::accumulate

```c++
accumulate(num.begin(), num.end(), 0)
```



## Lambda函数

1. `[captures](parameters){lambda_body};`

2. `captures`表示需要使用到的变量

   ```c++
   #include <iostream>
   int main()
   {
       int x = 123;
       int y = 456;
       auto mylambda = [x, y]() {std::cout << "X is: " << x << ", y is:" << y; };
           mylambda();
       }
   ```

3. 使用lambda作为算法函数

   ```c++
   #include <iostream>
   #include <vector>
   #include <algorithm>
   int main()
   {
       std::vector<int> v = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30 };
       auto counteven = std::count_if(std::begin(v), std::end(v),
       [](int x) {return x % 2 == 0; });
       std::cout << "The number of even vector elements is: " << counteven;
   }
   ```

   

# CMake

> 只有带main函数的文件会生成可执行程序，其余代码打包成库。
>
> 库分为静态库和共享库两种，前者以.a为后缀，后者以.so结尾。静态库每次调用会生成一个副本，动态库中有一个副本。

基本使用：

```cmake
mkdir build && cd build
cmake ../
make
```

CMakeLists.txt

```cmake
# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(HelloSLAM)

# 设置编译模式
set(CMAKE_BUILD_TYPE "Debug")

# 寻找所需要的库
# [COMPONENTS] [components…]：表示查找的包中必须要找到的组件(components），REQUIRED表示必须要找到的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable(helloSLAM helloSLAM.cpp)

# 添加hello库
add_library(hello libHelloSLAM.cpp)
# 共享库
add_library(hello_shared SHARED libHelloSLAM.cpp)

# 添加可执行程序调用hello库中函数
add_executable(useHello useHello.cpp)
# 将库文件链接到可执行程序上(需要有头文件)
target_link_libraries(useHello hello_shared)
# 或者将库文件链接到整个项目上
target_link_libraries($(CMAKE_PROJECT_NAME) hello_shared)
# 表示编译顺序（即listener的编译需要beginner_tutorials_generate_messages_cpp的依赖，因此先生成依赖项）
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

