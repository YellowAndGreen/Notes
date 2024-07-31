### 代码风格

1. 类以驼峰命名：class FooClass
2. 函数名以\_分割：def func_separated_by_underscores(*a):
3. 变量名小写：var = "lowercase"
4. 内部变量以\_开头：_var = "_single_leading_underscore"
5. 类私有变量以\_\_开头：__var = " __double_leading_underscore"
6. 不需要的变量为\_
7. 使用PEP8：加入vscode插件autopep

### 经验

+ range函数生成的是0到给定数字-1的序列，因此遍历数组只需要传入len即可，若有比较i和i+1，只需传入len-1即可
+ 列表切片是不会返回:后面数字对应索引值的！因此返回整个列表应该是[0:len]，虽然len值超出索引

#### 操作符

is 和 is not 在判断时比较的是两个变量引用对象的内存地址，而 == 和 != 则比较的是引用对象的值。判断一个值是不是None时，最好使用is和is not


```python
a = [1,2]
b = [1,2]
a is b,a==b
```




    (False, True)



#### 位运算

右移运算符（>>）

定义：将一个数的各二进制位全部右移若干位，正数左补0，负数左补1，右边丢弃。

例如：a=a>>2 将a的二进制位右移2位，左补0 或者 左补1得看被移数是正还是负。

***操作数每右移一位，相当于该数除以2。左移同理，等于乘2。计算后的数向下取整。***


```python
7>>2
```




    1




```python
12<<1
```




    24




```python
3<<2
```




    12

#### 包管理

```
pip freeze > requirements.txt
```




### 标准模块

#### argparse：解析命令行参数

argparse的使用简化成下面四个步骤

1：import argparse

2：parser = argparse.ArgumentParser()

3：parser.add_argument()

4：parser.parse_args()

上面四个步骤解释如下：首先导入该模块；然后创建一个解析对象；然后向该对象中添加你要关注的命令行参数和选项，每一个add_argument方法对应一个你要关注的参数或选项；最后调用parse_args()方法进行解析；解析成功之后即可使用。

下面的例子中输入`python test.py -h`会显示帮助文档，输入--addresses xxx会按照程序执行所需的操作**


```python
import argparse   #步骤一

def parse_args():
    """
    :return:进行参数的解析
    """
    description = "you should add those parameter"                   # 步骤二
    parser = argparse.ArgumentParser(description=description)        # 这些参数都有默认值，当调用parser.print_help()或者运行程序时由于参数不正确(此时python解释器其实也是调用了pring_help()方法)时，
                                                                     # 会打印这些描述信息，一般只需要传递description参数，如上。
    help = "The path of address"
    parser.add_argument('--addresses',help = help)                   # 步骤三，后面的help是我的描述
    args = parser.parse_args()                                       # 步骤四          
    return args

if __name__ == '__main__':
    args = parse_args()
    print(args.addresses)            #直接这么获取即可。
```



### 生成器

+ 最基础的生成器表达式的语法跟列表推导差不多，只是把方括号换成圆括号
+ 如果生成器表达式是一个函数调用过程中的唯一参数，那么不需要额外再用括号把它围起来。

```py
>>> symbols = '$¢£¥€¤'
>>> tuple(ord(symbol) for symbol in symbols) ➊
(36, 162, 163, 165, 8364, 164)
>>> import array
>>> array.array('I', (ord(symbol) for symbol in symbols)) ➋
array('I', [36, 162, 163, 165, 8364, 164])
```

+ 调用生成器函数会返回一个生成器对象（在函数体内有yield就会被识别为生成器函数）
+ 只有对生成器对象调用next时，才会真正执行函数体内代码
+ 当yield后，函数会暂停在yield的下面一行，下次调用next时接在下面执行
+ 生成器和迭代器的区别就在于不需要保存当前进度的状态，只需要接着去运行即可
+ 可以通过send方法在yield一个值后赋予yield值

+ 函数传参的时候使用生成器可以**减小内存占用**


```python
def sample():
    yield 'Is'
    yield 'Chicago'
    yield 'Not'
    yield 'Chicago?'
''.join(sample())
# 'IsChicagoNotChicago?'
```

**列表生成式和生成器的区别仅在于最外层的[]和()**

生成器不会直接计算出所有结果，只有通过g.next()或者for循环获得下一个返回值：


```python
g = (x * x for x in range(3))
for n in g:
    print(n)
```

**暂停示例：**

```python
# 函数生成器,yield会返回值
def odd():
    print('step 1')
    yield (1)
    print('step 2')
    yield(3)
    print('step 3')
    yield(5)
o = odd()
for i in o:
    print(i)
```

**yield from：**

+ 允许你将一个可迭代对象的元素直接“传递”给上层的迭代器或生成器。
+ 它的作用是将一个嵌套的可迭代对象（例如列表的列表）展平为一个单一的序列。

```py
class Node:
    def __init__(self, value):
        self._value = value
        self._children = []

    def __repr__(self):
        return 'Node({!r})'.format(self._value)

    def add_child(self, node):
        self._children.append(node)

    def __iter__(self):
        return iter(self._children)

    def depth_first(self):
        yield self
        for c in self:
            yield from c.depth_first()

# Example
if __name__ == '__main__':
    root = Node(0)
    child1 = Node(1)
    child2 = Node(2)
    root.add_child(child1)
    root.add_child(child2)
    child1.add_child(Node(3))
    child1.add_child(Node(4))
    child2.add_child(Node(5))

    for ch in root.depth_first():
        print(ch)
    # Outputs Node(0), Node(1), Node(3), Node(4), Node(2), Node(5)
```



### 变量

#### 字符串

- 使用**格式化字符串字面值**，要在字符串开头的引号/三引号前添加 `f` 或 `F` 。在这种字符串中，可以在 `{` 和 `}` 字符之间输入引用的变量，或字面值的 Python 表达式。

  ```
  >>> year = 2016
  >>> event = 'Referendum'
  >>> f'Results of the {year} {event}'
  'Results of the 2016 Referendum'
  ```

- 字符串的 [`str.format()`](https://docs.python.org/zh-cn/3.8/library/stdtypes.html#str.format) 方法需要更多手动操作。该方法也用 `{` 和 `}` 标记替换变量的位置，虽然这种方法支持详细的格式化指令，但需要提供格式化信息。

  ```
  >>> yes_votes = 42_572_654
  >>> no_votes = 43_132_495
  >>> percentage = yes_votes / (yes_votes + no_votes)
  >>> '{:-9} YES votes  {:2.2%}'.format(yes_votes, percentage)
  ' 42572654 YES votes  49.67%'
  ```

- **格式说明符**

使用 `:` 后跟 `.` 和所需的小数位数来设置小数点后的位数。例如，如果你想将数字格式化为保留两位小数，可以这样做：

```python
num = 3.1415926
formatted_str = f"{num:.2f}"
print(formatted_str)  # 输出: "3.14"
```

这里的 `.2f` 表示以浮点数格式显示，保留两位小数。

此外，如果需要设置宽度和精度，可以这样写：

```python
formatted_str = f"{num:10.3f}"  # 宽度为10，小数点后保留3位
print(formatted_str)  # 输出: "     3.142"
```

在这个例子中，`10.3f` 表示总宽度为10个字符，其中3位是小数点后的数字，其余的用空格填充。

**整数格式化**的例子：

1. **设置宽度**:
   使用数字指定宽度，如果整数位数小于这个宽度，将会用空格填充。

   ```python
   num = 123
   print(f"{num:5}")  # 输出: "  123"
   ```

2. **设置填充字符**:
   使用 `{:<填充字符>宽度}` 来指定填充字符和宽度。

   ```python
   print(f"{num:->10}")  # 输出: "123     "
   print(f"{num:0>10}")  # 输出: "000000123"
   ```

3. **设置对齐方式**:

   - `<` 表示左对齐（默认）
   - `>` 表示右对齐
   - `^` 表示居中对齐

   ```python
   print(f"{num:<10}")  # 输出: "123      "
   print(f"{num:>10}")  # 输出: "      123"
   print(f"{num:^10}")  # 输出: "   123   "
   ```

4. **添加小数点并填充零**:
   如果你想在整数后面添加小数点并填充零，可以使用 `:.0f` 格式化，这将把整数转换为浮点数并去掉小数部分。

   ```python
   print(f"{num:.0f}")  # 输出: "123."
   ```

5. **使用逗号分隔千位**:
   对于较大的整数，可以使用逗号作为千位分隔符。

   ```python
   big_num = 123456789
   print(f"{big_num:,}")  # 输出: "123,456,789"
   ```

#### global和nonlocal

`global` 关键字用于在函数内部声明变量为全局变量，允许对其全局状态进行修改；而 `nonlocal` 关键字用于在嵌套的函数中声明变量为最近封闭作用域中的非局部变量，允许对其值进行修改。

```py
def scope_test():
    def do_local():
        spam = "local spam"

    def do_nonlocal():
        nonlocal spam
        spam = "nonlocal spam"

    def do_global():
        global spam
        spam = "global spam"

    spam = "test spam"
    do_local()
    print("After local assignment:", spam)
    do_nonlocal()
    print("After nonlocal assignment:", spam)
    do_global()
    print("After global assignment:", spam)

scope_test()
print("In global scope:", spam)
```

示例代码的输出是：

```py
After local assignment: test spam
After nonlocal assignment: nonlocal spam
After global assignment: nonlocal spam
In global scope: global spam
```

#### 弱引用

在Python中，弱引用（weak reference）是一种对对象的引用，它**不会增加对象的引用计数**。这意味着，即使存在弱引用，当对象的其他所有强引用被删除时，对象仍然可以被垃圾回收器回收。弱引用主要用于避免内存泄漏，尤其是在管理缓存或处理对象集合时，这些对象可能会被外部频繁地创建和销毁。

弱引用的一个典型应用场景是缓存机制，例如，一个函数可能会缓存其结果以避免重复计算。如果使用强引用来存储这些结果，可能会导致内存泄漏，因为即使不再需要这些结果，它们仍然会被缓存对象引用而无法被回收。使用弱引用可以确保当缓存的对象不再被其他地方使用时，它们可以被自动回收。

Python提供了`weakref`模块来支持弱引用。以下是使用弱引用的一些基本示例：

1. 创建弱引用：
   ```python
   import weakref
   
   class MyClass:
       pass
   
   obj = MyClass()
   ref = weakref.ref(obj)
   ```

2. 检查弱引用是否仍然有效：
   ```python
   if ref() is not None:
       print("对象还存在")
   else:
       print("对象已经被垃圾回收")
   ```

3. 使用弱引用调用对象的方法：
   ```python
   if ref() is not None:
       ref().my_method()
   ```

4. 弱引用集合（`weakref.WeakSet`）和弱引用字典（`weakref.WeakValueDictionary`）：
   ```python
   refs = weakref.WeakSet()
   refs.add(obj)
   
   weak_dict = weakref.WeakValueDictionary()
   weak_dict['key'] = obj
   ```



### 函数

#### 文档功能

1. 在函数名下以"""开头结尾的注释作为文档
2. 可通过__doc__或者help(func)来获取注释


```python
def example():
    """a test da"""
    pass
example.__doc__
```




    'a test da'




```python
help(example)
```

    Help on function example in module __main__:
    
    example()
        a test da


​    

#### 使用`*`和`/`分隔

```py
def f(pos1, pos2, /, pos_or_kwd, *, kwd1, kwd2):
```

说明：

- `\`之前只能使用位置形参，`*`号之后均只能通过关键字调用
- 使用仅限位置形参，可以让用户无法使用形参名。形参名没有实际意义时，强制调用函数的实参顺序时，或同时接收位置形参和关键字时，这种方式很有用。
- 当形参名有实际意义，且显式名称可以让函数定义更易理解时，阻止用户依赖传递实参的位置时，才使用关键字。
- 对于 API，使用仅限位置形参，可以防止未来修改形参名时造成破坏性的 API 变动。


#### 类型注解

1. 参数通过:指定，返回值通过->指定
2. \_\_annotations\_\_可返回注解信息


```python
def fib(n: int) -> int:
    a, b = 0, 1
    for _ in range(n):
        b, a = a + b, b
    return a

fib(10)
```




    55




```python
fib.__annotations__
```




    {'n': int, 'return': int}




```python
from typing import Generator
def fib(n: int) -> Generator:
    a, b = 0, 1
    for _ in range(n):
        b, a = a + b, b
        yield a

a = fib(10)
list(a)
```




    [1, 1, 2, 3, 5, 8, 13, 21, 34, 55]



#### 函数结果缓存：lru_cache

+ 将函数输入和输出做为键值对保存


```python
from functools import lru_cache
import time
@lru_cache(maxsize=None)
def fib(n):
    if n < 2:
        return n
    return fib(n - 1) + fib(n - 2)

s = time.time()
_ = fib(521)
e = time.time(); e - s
```




    0.0



#### 函数的上下文管理器


```python
from contextlib import contextmanager

@contextmanager
def opening(filename, mode='r'):
   f = open(filename, mode)
   try:
      yield f
   finally:
      f.close()

with opening('example.txt') as fd:
   fd.read()
```

### 高阶函数

#### MAP / REDUCE


```python
#MAP：传入一个函数，一个列表，返回一个经过函数处理的Iterator

#将数字转换为字符串
a = list(map(str, [1, 2, 3, 4, 5, 6, 7, 8, 9]))
```

REDUCE：传入一个函数,一个列表和初始值

若无初始值则将第一个作为结果，第二个作为参数传入函数


```python
# 不同在于REDUCE把结果继续和序列的下一个元素做累积计算

#把序列[1, 3, 5, 7, 9]变换成整数13579
from functools import reduce
def fn(x, y):
    return x * 10 + y
reduce(fn, [1, 3, 5, 7, 9])
```




    13579




```python
def fn(x, y):
    return x * 10 + y
reduce(fn, [1, 3, 5, 7, 9],1)
```




    113579



#### 匿名函数LAMBDA


```python
list(map(lambda x: x * x, [1, 2, 3, 4, 5, 6, 7, 8, 9]))
```




    [1, 4, 9, 16, 25, 36, 49, 64, 81]



### 时间

#### time模块

+ time.time()  获取当前时间戳
+ time.sleep()  推迟调用线程的运行


```python
import time
start = time.time()
a=1
for i in range(1000000):
    a*=i
end = time.time()
print(end-start)
```

    0.07477164268493652


#### timeit 模块

测试一行代码的运行时间，默认测试100万次，可指定number


```python
import timeit 
t2 = timeit.Timer('x=range(1000)') 
t2.timeit(number=10000)
```




    0.0016464000000269152



#### 魔术指令timeit

+ %timeit可以用于得到***一行代码***的运行时间，%%time可以用于得到***一个cell中***的代码运行时间。
+ %timeit -n  -r  -t|-c -q -p -o
+ -n：在一次循环中执行N次给定的代码
+ -r: 代码重复r个循环
+ -t：使用time.time作为基础来计算时间，是Unix的默认方式
+ -c：使用time.clock作为基础来计算时间，是windows的默认方式，返回cpu时间
+ -p ：结果保留P个有效数字
+ -o：加上-o后可以将结果赋值给一个变量，就可以直接调用%timeit的结果了。


```python
%%timeit -n 10 -r 5 -o
import numpy as np
print(np.random.randint(10))
```

### 进程和线程

+ 对于操作系统来说，一个任务就是一个进程（Process），进程内的这些“子任务”称为线程（Thread）


```python
# 一个CPU密集型任务
def count1(n):
    while n>0:
        n-=1
# 一个I/O密集型任务，CPU处理时间很快多数时间在等待I/O阻塞
def count2():
    time.sleep(0.01)
```

#### 多线程


```python
from threading import Thread
def test_thread():
    t1=Thread(target=count1,args=(100000,))  # 逗号不能少
    t2=Thread(target=count1,args=(100000,))
    t1.start()
    t2.start()
    if t1.is_alive():  # 判断进程是否运行
        print('Still running')
    else:
        print('Completed')
    t1.join()  # 主线程会等待t1执行结束
    t2.join()
    
# 后台线程
t = Thread(target=countdown, args=(10,), daemon=True)
t.start()
```

#### 多进程

**`multiprocessing`在程序内部执行并行**


```python
from multiprocessing import Process
def test_process():
    t1=Process(target=count1,args=(100000,))
    t2=Process(target=count1,args=(100000,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
```

**`subprocess`用于运行和管理外部进程**

```py
import subprocess

# 创建子进程但不等待其完成
process = subprocess.Popen(["ping", "localhost"], stdout=subprocess.PIPE)

# 从子进程获取输出
output, _ = process.communicate()
print(output.decode())

# 使用shell模式
proc = subprocess.Popen('bash rosplay.sh  > ./log/rosplay.log 2>&1', env=copy_env, shell=True)
```




### 面向对象编程

#### 基础使用


```python
class Student(object):

    def __init__(self, name, score):   # 初始化函数，第一个参数必须是self
        self.__name = name             # 在类对象属性前加两个下横杠使此属性变为私有变量，不能再外部直接访问，只能通过对象方法获取
        self.__score = score

    def print_score(self):
        print('%s: %s' % (self.__name, self.__score))
```


```python
a = Student("11","12")
```


```python
a.print_score()
```

    11: 12


#### 继承与多态

```
class Animal(object):
    def run(self):
        print('Animal is running...')
class Dog(Animal):  # 子类自动拥有父类的方法
    pass
```

##### 获取继承关系 class.mro()


```python
class A(object):
    def say(self):
        print('A')
class B(object):
    def say(self):
        print('b')
class C(A,B):
    pass
c = C()
c.say()
```

    A



```python
C.mro()
```




    [__main__.C, __main__.A, __main__.B, object]



#### 方法

##### 类方法和静态方法

+ 静态方法类和实例均能调用，类方法只能类调用，实例方法只能实例调用


```python
class example(object):
    @classmethod
    def clsmethod(cls):
        print("I am classmethod")
    @staticmethod
    def stmethod():
        print("I am staticmethod")
    def instmethod(self):
        print("I am instancemethod")
ex = example()
ex.classmethod()
```


    ---------------------------------------------------------------------------
    
    AttributeError                            Traceback (most recent call last)
    
    <ipython-input-103-e1e5493ebe56> in <module>
          9         print("I am instancemethod")
         10 ex = example()
    ---> 11 ex.classmethod()


    AttributeError: 'example' object has no attribute 'classmethod'



```python
example.stmethod()
```

    I am staticmethod



```python
example.instmethod()
```


    ---------------------------------------------------------------------------
    
    TypeError                                 Traceback (most recent call last)
    
    <ipython-input-101-2b44b85c2540> in <module>
    ----> 1 example.instmethod()


    TypeError: instmethod() missing 1 required positional argument: 'self'



```python
ex.stmethod(),example.stmethod()
```

    I am staticmethod
    I am staticmethod





    (None, None)



##### 抽象方法

+ 定义一个不需要实现的方法，类似于接口
+ 继承该类的子类必须实现该抽象方法


```python
from abc import ABCMeta, abstractmethod
class base(object):
  __metaclass__ = ABCMeta
  @abstractmethod
  def absmethod(self):
    """ Abstract method """

class example(base):
  def absmethod(self):
    print("abstract")

ex = example()
ex.absmethod()
```

    abstract


##### 方法链

使用className.method1().method2.method3()的方式调用方法


```python
class Student():
    def __init__(self,name):
        self.name = name
    def addPrefix(self,pre):
        self.name = pre+self.name
        return self
    def addSuffix(self,suf):
        self.name = self.name+suf
        return self
s = Student("lihua")
# 两种方法链调用方式
# s.addPrefix("aaa").addSuffix("bbb")
(
    s.addPrefix("aaa")
     .addSuffix("bbb")
)
s.name
```




    'aaalihuabbb'



#### 获取对象信息

##### isinstance判断类型

isinstance() 与 type() 区别：

type() 不会认为子类是一种父类类型，不考虑继承关系。

isinstance() 会认为子类是一种父类类型，考虑继承关系。


```python
class Person():
    pass
class Student(Person):
    pass

p = Person()
s = Student()
isinstance(s,Person), type(s)
```




    (True, __main__.Student)



##### dir()：获取一个对象所有属性和方法

+ 若不指定对象，则返回当前定义域里所有的变量名


```python
dir("abc")
```




    ['__add__',
     '__class__',
     '__contains__',
     '__delattr__',
     '__dir__',
     '__doc__',
     '__eq__',
     '__format__',
     '__ge__',
     '__getattribute__',
     '__getitem__',
     '__getnewargs__',
     '__gt__',
     '__hash__',
     '__init__',
     '__init_subclass__',
     '__iter__',
     '__le__',
     '__len__',
     '__lt__',
     '__mod__',
     '__mul__',
     '__ne__',
     '__new__',
     '__reduce__',
     '__reduce_ex__',
     '__repr__',
     '__rmod__',
     '__rmul__',
     '__setattr__',
     '__sizeof__',
     '__str__',
     '__subclasshook__',
     'capitalize',
     'casefold',
     'center',
     'count',
     'encode',
     'endswith',
     'expandtabs',
     'find',
     'format',
     'format_map',
     'index',
     'isalnum',
     'isalpha',
     'isascii',
     'isdecimal',
     'isdigit',
     'isidentifier',
     'islower',
     'isnumeric',
     'isprintable',
     'isspace',
     'istitle',
     'isupper',
     'join',
     'ljust',
     'lower',
     'lstrip',
     'maketrans',
     'partition',
     'replace',
     'rfind',
     'rindex',
     'rjust',
     'rpartition',
     'rsplit',
     'rstrip',
     'split',
     'splitlines',
     'startswith',
     'strip',
     'swapcase',
     'title',
     'translate',
     'upper',
     'zfill']



##### 获取实例的类名 name.__class__.__name__


```python
class ExampleClass(object):
    pass

ex = ExampleClass()
ex.__class__.__name__
```




    'ExampleClass'



##### 判断继承关系issubclass


```python
class Example(object):
    def __init__(self):
        self.name = "ex"
    def printex(self):
        print("This is an Example")

issubclass(Example, object)
```




    True



### 高级面向对象

定义了一个class，创建了一个class的实例后，可以给该实例绑定任何属性和方法，也可以直接对该类绑定

```
>>> def set_age(self, age): # 定义一个函数作为实例方法
...     self.age = age
...
>>> from types import MethodType
>>> s.set_age = MethodType(set_age, s) # 给实例绑定一个方法
>>> s.set_age(25) # 调用实例方法
>>> s.age # 测试结果
25
```

#### \_\_init\_\_()：初始化对象中默认参数进行重载的坑

+ python函数可以通过设置默认参数，从而实现可变参数值，叫做重载
+ init函数中的默认值若传入的是一个mutable对象，那么不指定该参数时，所有实例的此参数都会指向同一个内存id，修改其中一个，其他的也会变化
+ mutable对象的引用是指针传递，而immutable对象的引用是值传递
+ 解决此问题的方法是将默认值设置为None，在init中通过判断来赋予默认值


```python
class Student(object):
    def __init__(self,names=[]):
        self.names = names
s1=Student()
s2=Student()
s1.names.append(1)
s2.names.append(2)
s1.names
```




    [1, 2]




```python
class Student(object):
    def __init__(self,names=None):
        if names is None:
            self.names = []
        else:
            self.names = names
s1=Student()
s2=Student()
s1.names.append(1)
s2.names.append(2)
s1.names,s2.names
```




    ([1], [2])



#### \_\_super\_\_

1. 第一个参数表示从哪个类的MRO开始往下寻找\_\_init\_\_等所使用的函数
2. 无参数时默认使用所处类作为第一个参数，以所处函数的第一个参数作为第二个参数


```python
class Animal():
    def __init__(self):
        print("Animal初始化了")

class Human(Animal):
    def __init__(self):
        print("Human初始化了")

class Student(Human):
    def __init__(self,names=None):
        super().__init__()
        super(Student,self).__init__()
        super(Human,self).__init__()
        print("Student初始化了")

s1=Student()
```

    Human初始化了
    Human初始化了
    Animal初始化了
    Student初始化了


#### \_\_slots\_\_()：只允许实例添加某些属性

+ 可以节省内存


```python
class Student(object):
    __slots__ = ('name', 'set_age') # 用tuple定义允许绑定的属性名称
```


```python
s = Student()
```


```python
s.age = 1
```

属性无法绑定


```python
s.year = 2
```


    ---------------------------------------------------------------------------
    
    AttributeError                            Traceback (most recent call last)
    
    <ipython-input-8-1b62f9eb30b6> in <module>
    ----> 1 s.year = 2


    AttributeError: 'Student' object has no attribute 'year'


方法无法绑定


```python
def set_age(self, age): 
    self.age = age
from types import MethodType
s.set_age = MethodType(set_age, s) # 给实例绑定一个方法
```


    ---------------------------------------------------------------------------
    
    AttributeError                            Traceback (most recent call last)
    
    <ipython-input-9-05f53e533c5b> in <module>
          2     self.age = age
          3 from types import MethodType
    ----> 4 s.set_age = MethodType(set_age, s) # 给实例绑定一个方法


    AttributeError: 'Student' object has no attribute 'set_age'


修改元组后可绑定


```python
def set_age(self, age): 
    self.age = age
s = Student()
from types import MethodType
s.set_age = MethodType(set_age, s) # 给实例绑定一个方法
```

#### @property：使用装饰器将一个方法当做一个属性来使用

+ 也可以将@property作为计算属性使用


```python
class Student(object):

    @property
    def score(self):    # 相当于getter方法
        return self._score

    @score.setter        # 相当于setter方法
    def score(self, value):
        if not isinstance(value, int):
            raise ValueError('score must be an integer!')
        if value < 0 or value > 100:
            raise ValueError('score must between 0 ~ 100!')
        self._score = value
    @score.deleter
    def score(self):
        del self._score
```


```python
s = Student()
s.score = 1
s.score
```




    1




```python
s.score = 2
s.score
```




    2




```python
del s.score
s.score
```


    ---------------------------------------------------------------------------
    
    AttributeError                            Traceback (most recent call last)
    
    <ipython-input-90-a24d22fa8a4f> in <module>
          1 del s.score
    ----> 2 s.score


    <ipython-input-84-8e9af9da06cf> in score(self)
          3     @property
          4     def score(self):    # 相当于getter方法
    ----> 5         return self._score
          6 
          7     @score.setter        # 相当于setter方法


    AttributeError: 'Student' object has no attribute '_score'


计算属性


```python
class Student(object):

    @property
    def score(self):    # 相当于getter方法
        return self._score*2
    @score.setter        # 相当于setter方法
    def score(self, value):
        self._score = value
s = Student()
s.score = 2
s.score
```




    4



#### \_\_iter\_\_：理解迭代器和可迭代对象的概念

+ \_\_iter\_\_()方法，该方法返回一个迭代对象，然后，Python的for循环就会不断调用该迭代对象的\_\_next\_\_()方法拿到循环的下一个值，直到遇到StopIteration错误时退出循环。
+ 可迭代对象可以没有next方法，看作是一个数据容器，可以不知道数据next数到哪里去了
+ 可迭代对象的iter必须返回一个迭代器
+ 可迭代对象要么有getitem，要么有iter
+ 迭代器必须有next方法
+ 官方规定迭代器本身也必须是可迭代对象，即iter返回自己，否则可能出错


```python
class Fib(object):    # 斐波那契数列
    def __init__(self):
        self.a, self.b = 0, 1 # 初始化两个计数器a，b

    def __iter__(self):
        return self # 实例本身就是迭代对象，故返回自己

    def __next__(self):
        self.a, self.b = self.b, self.a + self.b # 计算下一个值
        if self.a > 100: # 退出循环的条件
            raise StopIteration()
        return self.a # 返回下一个值
```


```python
for n in Fib():
    print(n)
```

    1
    1
    2
    3
    5
    8
    13
    21
    34
    55
    89


#### \_\_contains\_\_

实现了contains的类可以使用in关键字


```python
class Stack:

    def __init__(self):
        self.__list = []

    def push(self, val):
        self.__list.append(val)

    def pop(self):
        return self.__list.pop()

    def __contains__(self, item):
        return True if item in self.__list else False

stack = Stack()
stack.push(1)
print(1 in stack)
print(0 in stack)
```

    True
    False


#### \_\_getitem\_\_


```python
# 像list那样按照下标取出元素
class Fib(object):
    def __getitem__(self, n):
        a, b = 1, 1
        for x in range(n):
            a, b = b, a + b
        return a
```


```python
f = Fib()
f[0]
```




    1



#### \_\_getattr\_\_

当调用不存在的属性时，比如score，Python解释器会试图调用\_\_getattr\_\_(self, 'score')来动态返回一个属性


```python
class Student(object):

    def __init__(self):
        self.name = 'Michael'

    def __getattr__(self, attr):
        if attr=='score':
            return 99
```

#### \_\_call\_\_：在实例中调用自己的属性和方法

+ 可通过callable来判断能否调用


```python
class Student(object):
    def __init__(self, name):
        self.name = name

    def __call__(self):
        print('My name is %s.' % self.name)
```


```python
s = Student('Michael')
s()
```

    My name is Michael.



```python
callable(s),callable('12')
```




    (True, False)



#### \_\_repr\_\_： 方法是类的实例化对象用来做“自我介绍”的方法

默认情况下，它会返回当前对象的“类名+object at+内存地址”，而如果对该方法进行重写，可以为其制作自定义的自我描述信息。


```python
class CLanguage:
    def __init__(self):
        self.name = "C语言中文网"
        self.add = "http://c.biancheng.net"
    def __repr__(self):
        return "CLanguage[name="+ self.name +",add=" + self.add +"]"
    def __str__(self):
        return "21CLanguage[name="+ self.name +",add=" + self.add +"]"
clangs = CLanguage()
print(clangs,)
```

    21CLanguage[name=C语言中文网,add=http://c.biancheng.net]


####  \_\_str\_\_： 同上一个作用，但不需要加（）

#### 富比较方法

+ object.lt(self, other)
+ object.le(self, other)
+ object.eq(self, other)
+ object.ne(self, other)
+ object.gt(self, other)
+ object.ge(self, other)

> x<y 调用 x.__lt__(y)、x<=y 调用 x.__le__(y)、x==y 调用 x.__eq__(y)、x!=y 调用 x.__ne__(y)、x>y 调用 x.__gt__(y)、x>=y 调用 x.__ge__(y)

+ 实现了上述\_\_lt\_\_方法的的类实例可以直接使用sorted


```python
class Stu:
    def __init__(self,val):
        self.val = val
    def __lt__(self,other):
        return self.val < other.val
s1,s2,s3 = Stu(3),Stu(4),Stu(1)
for i in sorted([s1,s2,s3]):
    print(i.val)
```

    1
    3
    4



```python
s1<s3
```




    False



#### 枚举类


```python
from enum import Enum

Month = Enum('Month', ('Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'))
```


```python
Month.Apr
```




    <Month.Apr: 4>




```python
for name, member in Month.__members__.items():
    print(name, '=>', member, ',', member.value)
```

    Jan => Month.Jan , 1
    Feb => Month.Feb , 2
    Mar => Month.Mar , 3
    Apr => Month.Apr , 4
    May => Month.May , 5
    Jun => Month.Jun , 6
    Jul => Month.Jul , 7
    Aug => Month.Aug , 8
    Sep => Month.Sep , 9
    Oct => Month.Oct , 10
    Nov => Month.Nov , 11
    Dec => Month.Dec , 12


#### 装饰器

+ 加上装饰器后，调用函数相当于调用装饰器函数返回的wrapper函数


```python
import time

def timeit(f):
    def wrapper(*args,**kwargs):   #此处是传入的参数
        start = time.time()
        ret = f(*args,**kwargs)
        print(f'共计{time.time()-start}s')
        return ret
    return wrapper
```


```python
@timeit
def count(n):
    ret = 1
    for i in range(1,n):
        ret*=i
    return 
count(100000)
```

    共计3.23993182182312s


##### 作为装饰器的类

下面的例子的add(a,b)等价于

```
t = Timer1()
add2 = t(add)
add2(a,b)
```


```python
import time
from datetime import datetime
class Timer1:
    def __init__(self,curtime=False):
        self.curtime = curtime
    def __call__(self,func):    # 调用实例方法
        def wrapper(*args,**kwargs):   # 返回包装函数
            if self.curtime:
                return datetime.now()
            start = time.time()
            ret = func(*args,**kwargs)
            print(f'共计{time.time()-start}s')
            return ret
        return wrapper
@Timer1(False)
def add(a,b):
    return a**b
```


```python
add(9780,190)
```

    共计0.0s





    146009935678515608836623623653006338089514492205738947686200317367504800076206500761216723772421058412842984685681563058067824770567637981111222732776957323460192102541073169841842501023701818309022952627438278688079503679607229931344597096770095387297990823692220798373273491334948256888326298669840245885960182742553827638808395989367144363704881158546828344553535446226688901516063170351509798338963730080045893622559914579108693645082351450714320182614989932368395320995560563631367909173005426012341560983290054008921194361815505893056298945231514489507492910858240000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000



##### 一个类的装饰器

例：为每一个有此装饰器的类添加\_\_str\_\_方法


```python
def appendstr(cls):
    def __str__(self):
        return f'名字为{self.name}'
    cls.__str__ = __str__
    return cls

@appendstr
class Student:
    def __init__(self,name):
        self.name = name
```


```python
s = Student('paul')
print(s)
```

    名字为paul


##### 使用@wraps装饰器解决函数的文档变成wrapper函数的问题


```python
from functools import wraps
def decorator(func):
    """this is decorator __doc__"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        """this is wrapper __doc__"""
        print("this is wrapper method")
        return func(*args, **kwargs)
    return wrapper

@decorator
def test():
    """this is test __doc__"""
    print("this is test method")

print("__name__: ", test.__name__)
print("__doc__:  ", test.__doc__)

```

    __name__:  test
    __doc__:   this is test __doc__



#### 抽象基类

`from abc import ABC, abstractmethod` 是 Python 编程语言中使用抽象基类（Abstract Base Classes，简称 ABC）的一种方式。在 Python 中，`abc` 模块提供了基础设施，使得开发者可以创建抽象基类，这些类不能被实例化，但可以作为其他类的基类，以确保派生类实现了某些方法。

1. `ABC` 是一个元类，它可以用来创建抽象基类。当你将一个类继承自 `ABC` 时，这个类就成为了一个抽象基类。

2. `abstractmethod` 是一个装饰器，用于将方法声明为抽象方法。抽象方法是一种没有实现体的方法，它必须在任何非抽象的子类中被重写。如果子类没有重写这些抽象方法，那么子类也会被认为是抽象的，不能被实例化。

下面是一个简单的例子来说明如何使用 `ABC` 和 `abstractmethod`：

```python
from abc import ABC, abstractmethod

# 创建一个抽象基类
class MyAbstractClass(ABC):
    
    # 声明一个抽象方法
    @abstractmethod
    def my_method(self):
        pass

# 子类必须实现抽象方法
class MyConcreteClass(MyAbstractClass):
    
    def my_method(self):
        print("Method implemented in MyConcreteClass")

# 尝试实例化抽象基类会抛出异常
# my_abstract_instance = MyAbstractClass()  # 这会抛出 TypeError

# 实例化实现了抽象方法的子类是允许的
my_concrete_instance = MyConcreteClass()
my_concrete_instance.my_method()  # 输出: Method implemented in MyConcreteClass
```

在这个例子中，`MyAbstractClass` 是一个抽象基类，它包含了一个抽象方法 `my_method`。任何继承自 `MyAbstractClass` 的子类都必须实现 `my_method` 方法，否则它们也将是抽象类。`MyConcreteClass` 是一个实现了 `my_method` 方法的非抽象类，因此可以被实例化。


### 错误处理

#### try except finally

当我们认为某些代码可能会出错时，就可以用try来运行这段代码，如果执行出错，则后续代码不会继续执行，而是直接跳转至错误处理代码，即except语句块，执行完except后，如果有finally语句块，则执行finally语句块，至此，执行完毕。


```python
try:
    print('try...')
    r = 10 / 0
    print('result:', r)
except ZeroDivisionError as e:
    print('except:', e)
finally:
    print('finally...')
print('END')
```

    try...
    except: division by zero
    finally...
    END


#### assert断言

assert的意思是，表达式n != 0应该是True，否则，根据程序运行的逻辑，后面的代码肯定会出错。

如果断言失败，assert语句本身就会抛出AssertionError：


```python
def foo(s):
    n = int(s)
    assert n != 0, 'n is zero!'
    return 10 / n

def main():
    foo('0')
```

### 文件与系统模块

#### OS模块

1. os.name——判断现在正在实用的平台，Windows 返回 ‘nt'; Linux 返回’posix'

2. os.getcwd()——得到当前工作的目录。

3. os.listdir()——指定所有目录下所有的文件和目录名。例：
4. os.remove()——删除指定文件

5. os.rmdir()——删除指定目录

6. os.mkdir()——创建目录
7. 这样只能建立一层，要想递归建立可用：os.makedirs()
8. os.system()——执行shell命令。
9. os.chdir()——改变目录到指定目录
10. os.path.join(path, name)——连接目录和文件名。
11. os.path.isfile()——判断指定对象是否为文件。是返回True,否则False

12. os.path.isdir()——判断指定对象是否为目录。是True,否则False。

13. os.environ()——返回返回当前环境变量列表

14. `os.system(command)`: 执行指定的 shell 命令。


#### shutil模块

1. **shutil.copy(src, dst)**:
   复制单个文件。copy2能够保留修改时间等元数据。

   ```python
   import shutil
   shutil.copy('/path/to/source_file', '/path/to/destination_file')
   ```

2. **shutil.rmtree(path)**:
   递归删除目录及其内容。

   ```python
   import shutil
   shutil.rmtree('/path/to/directory')
   ```

3. **shutil.move(src, dst)**:
   移动文件或目录。

   ```python
   import shutil
   shutil.move('/path/to/source', '/path/to/destination')
   ```

4. **shutil.copytree(src, dst, symlinks=False)**:
   递归复制整个目录树。

   ```python
   import shutil
   shutil.copytree('/path/to/source_directory', '/path/to/destination_directory')
   ```

5. **shutil.make_archive(base_name, format, root_dir=None, base_dir=None)**:
   创建归档文件。

   ```python
   import shutil
   shutil.make_archive('archive_name', 'zip', '/path/to/directory_to_zip')
   ```



#### 文件读写

1. **open(file, mode='r', buffering=-1, encoding=None, errors=None, newline=None, closefd=True, opener=None)**:
   - `file`: 文件路径或文件描述符。
   - `mode`: 打开文件的模式，常见的有：
     - `'r'`: 读取模式，默认值。
     - `'w'`: 写入模式，会覆盖文件内容。
     - `'a'`: 追加模式，写入数据到文件末尾。
     - `'b'`: 二进制模式，与文本模式相对。
     - `'+'`: 更新模式，可以读取也可以写入。
   - `encoding`: 指定文件的编码。
   - `errors`: 指定如何处理编码错误。
   - `newline`: 控制何时写入换行符。
   - `closefd`: 是否在文件操作完成后关闭文件描述符。
   - `opener`: 指定一个可调用对象来打开文件。

2. **read([size])**:
   从文件中读取数据，`size` 指定要读取的字节数或字符数。

3. **readline([size])**:
   读取文件的一行，`size` 指定最大字节数或字符数。

4. **readlines([sizehint])**:
   读取文件的所有行，返回一个包含每行作为元素的列表。

5. **write(str)**:
   将字符串 `str` 写入文件。

6. **writelines(sequence)**:
   将序列 `sequence` 中的每个字符串写入文件。

7. **seek(offset[, whence])**:
   设置文件当前位置，`offset` 是偏移量，`whence` 指定偏移基准点（0 为文件开头，1 为当前位置，2 为文件末尾）。

8. **tell()**:
   返回文件当前位置。

9. **close()**:
   关闭文件，释放系统资源。

10. **flush()**:
    刷新文件的缓冲区。

11. **truncate([size])**:
    截断文件到 `size` 字节或字符，如果未指定 `size`，则截断到当前文件位置。

12. **isatty()**:
    判断文件是否是一个终端。

以下是一些文件读写的示例：

```python
# 打开文件
with open('example.txt', 'w') as file:
    file.write('Hello, World!\n')

# 读取文件
with open('example.txt', 'r') as file:
    content = file.read()
    print(content)

# 逐行读取文件
with open('example.txt', 'r') as file:
    for line in file:
        print(line, end='')

# 写入多行到文件
lines = ['First line.', 'Second line.', 'Third line.']
with open('example.txt', 'w') as file:
    file.writelines(lines)
```

#### 系统sys模块

+ 获取引用计数  sys.getrefcount(a)
+ 获取python版本  sys.version    也可使用python -V
+ 获取对象的内存字节占用量  sys.getsizeof()

`sys` 是 Python 的一个标准库模块，提供与 Python 解释器及其环境交互的功能。它经常用于访问与 Python 解释器相关的变量和函数，执行系统命令，以及处理脚本参数等。以下是 `sys` 模块中一些常用的组件和功能：

1. **sys.argv**:
   - 一个列表，包含命令行参数。第一个元素是脚本名称，随后的元素是传递给脚本的参数。
2. **sys.exit(*args, **kwargs)**:
   - 退出程序。如果提供参数，可以指定退出状态码。
3. **sys.path**:
   - 一个字符串列表，表示 Python 解释器搜索模块的路径。
4. **sys.modules**:
   - 一个字典，包含已经加载的模块。
5. **sys.stdin, sys.stdout, sys.stderr**:
   - 分别是标准输入、标准输出和标准错误输出的文件对象。
6. **sys.maxsize**:
   - 表示可以在 Python 中使用的最大的整数。
7. **sys.getsizeof(obj)**:
   - 返回对象 `obj` 的内存大小。
9. **sys.exc_info()**:
   - 返回一个元组，包含当前的异常信息，如果没有异常则返回 `None`。
10. **sys.platform**:
    - 一个字符串，表示正在运行的操作系统平台。
11. **sys.version**:
    - 包含 Python 解释器的版本信息。
12. **sys.executable**:
    - Python 解释器的路径。

### CookBook

#### 以迭代器形式赋值给N个变量


```python
data = [ 'ACME', 50, 91.1, (2012, 12, 21) ]
name, shares, price, date = data
```


```python
name, shares
```




    ('ACME', 50)



#### 占位符变量_

#### *号符

+ 将列表分为多个单变量参数
+ 在赋值时为一个可变列表，也可以操作字符串
+ 使用zip(*[...])将二维列表转置，但是个数只有子列表长度


```python
a=[1,2,3]
b = [2,3,4]
[*a,*b]
```




    [1, 2, 3, 2, 3, 4]




```python
record = ('Dave', 'dave@example.com', '773-555-1212', '847-555-1212')
name, email, *phone_numbers = record
phone_numbers
```




    ['773-555-1212', '847-555-1212']




```python
records = [
    ('foo', 1, 2),
    ('bar', 'hello'),
    ('foo', 3, 4),
]

def do_foo(x, y):
    print('foo', x, y)

def do_bar(s):
    print('bar', s)

for tag, *args in records:
    if tag == 'foo':
        do_foo(*args)
    elif tag == 'bar':
        do_bar(*args)
```

    foo 1 2
    bar hello
    foo 3 4



```python
line = 'nobody:*:-2:-2:Unprivileged User:/var/empty:/usr/bin/false'
uname, *fields, homedir, sh = line.split(':')
uname,homedir
```




    ('nobody', '/var/empty')




```python
list(zip(*['ab','cd','abc','da']))
```




    [('a', 'c', 'a', 'd'), ('b', 'd', 'b', 'a')]




```python
list(zip("ab", "a",'abc','da'))
```




    [('a', 'a', 'a', 'd')]



#### 数字

1. round:舍入，可以指定舍入到的位数

> 当一个值刚好在两个边界的中间的时候， round 函数返回离它最近的偶数。 也就是说，对1.5或者2.5的舍入运算都会得到2。
>
> 2. 无穷大和inf：使用float创建  float('inf') float('-inf')  float('nan')

##### 使用decimal进行精确的浮点数运算


```python
from decimal import Decimal
a = Decimal('1.1')
b = Decimal('2.1')
a+b
```




    Decimal('3.2')



控制数字位数和精度


```python
from decimal import localcontext
with localcontext() as ctx:
    ctx.prec = 10
    print(b/a)
```

    1.909090909


使用math.fsum()解决丢失的“1”


```python
nums = [1.23e+18, 1, -1.23e+18]
sum(nums)
```




    0.0




```python
import math
math.fsum(nums)
```




    1.0



##### 随机数

1. random.choice()：从一个序列中随机的抽取一个元素
2. random.sample()：从一个序列中随机的抽取N个元素
3. random.shuffle():打乱顺序，会改变原数组
4. random.randint()：生成随机整数
5. random.random()：生成0到1范围内均匀分布的浮点数


```python
import random
values = [1, 2, 3, 4, 5, 6]
random.choice(values),random.choices(values,k=2),random.sample(values,2)
```




    (2, [1, 1], [4, 6])




```python
random.shuffle(values)
values
```




    [5, 2, 4, 1, 3, 6]



#### 列表操作

##### 切片对象

可以用来形象化表明切片区域的用途，同时可复用

```py
>>> invoice = """
... 0.....6................................40........52...55........
30 ｜ 第2 章
... 1909 Pimoroni PiBrella $17.50 3 $52.50
... 1489 6mm Tactile Switch x20 $4.95 2 $9.90
... 1510 Panavise Jr. - PV-201 $28.00 1 $28.00
... 1601 PiTFT Mini Kit 320x240 $34.95 1 $34.95
... """
>>> SKU = slice(0, 6)
>>> DESCRIPTION = slice(6, 40)
>>> UNIT_PRICE = slice(40, 52)
>>> QUANTITY = slice(52, 55)
>>> ITEM_TOTAL = slice(55, None)
>>> line_items = invoice.split('\n')[2:]
>>> for item in line_items:
... print(item[UNIT_PRICE], item[DESCRIPTION])
...
$17.50 Pimoroni PiBrella
$4.95 6mm Tactile Switch x20
$28.00 Panavise Jr. - PV-201
$34.95 PiTFT Mini Kit 320x240
```



##### 列表生成式


```python
# 生成整数的平方数组
a= [n*n for n in range(1,11)]
print(a)
```

    [1, 4, 9, 16, 25, 36, 49, 64, 81, 100]



```python
# 字符串整理，可以使用函数
d = {'x': 'A', 'y': 'B', 'z': 'C' }
[k + '=' + v for k, v in d.items()]
```




    ['x=A', 'y=B', 'z=C']




```python
# 有判断功能

#此处不能使用else
a = [x for x in range(1, 11) if x % 2 == 0] 
#此处可以使用else
b = [x if x % 2 == 0 else -x for x in range(1, 11)]

print(a,b)
```

    [2, 4, 6, 8, 10] [-1, 2, -3, 4, -5, 6, -7, 8, -9, 10]



##### 初始化

+ 重复N次：[1]*10
+ 若是可变对象，则*重复只会复制指针
+ 使用列表表达式重复可以避免上述问题


```python
[1]*10
```




    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]




```python
m = [ [1,2,3,4,5]] *3
m
```




    [[1, 2, 3, 4, 5], [1, 2, 3, 4, 5], [1, 2, 3, 4, 5]]




```python
m[0][1] = 10
m[2][4] = 20
m
```




    [[1, 10, 3, 4, 20], [1, 10, 3, 4, 20], [1, 10, 3, 4, 20]]




```python
m = [[1,2,3,4,5] for _ in range(3)]
m[0][1] = 10
m[2][4] = 20
m
```




    [[1, 10, 3, 4, 5], [1, 2, 3, 4, 5], [1, 2, 3, 4, 20]]



##### 间隔切片

+ ::num表示以num为间隔切片
+ start:end:step表示从头到尾以step切片
+ start可以省略，但是:不能省略


```python
a = [1, 2, 3, 4, 5]
a[::2]
```




    [1, 3, 5]




```python
a = [1, 2, 3, 4, 5]
a[1:4:2]
```




    [2, 4]




```python
a[::-1]
```




    [5, 4, 3, 2, 1]




```python
a[3::-1]
```




    [4, 3, 2, 1]




```python
a[:3:2]
```




    [1, 3]



##### 扩展列表 ls.extend(ls2)


```python
a = [1,2,3]
b = [4,5]
a.extend(b)
a
```




    [1, 2, 3, 4, 5]



##### 列表分组拆分zip()

+ 拆分的长度为最短的列表长度
+ 若需要以最长的列表长度对齐，则使用from itertools import zip_longest


```python
a,b,c = [1,2,3],[4,5,6,],[1]
list(zip(a,b,c))
```




    [(1, 4, 1)]




```python
from itertools import zip_longest
list(zip_longest(a,b,c ))
```




    [(1, 4, 1), (2, 5, None), (3, 6, None)]



##### 数组按间隔n分组


```python
def chunk(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i+n]

a = [1, 2, 3, 4, 5, 6, 7, 8]
list(chunk(a, 3))
```




    [[1, 2, 3], [4, 5, 6], [7, 8]]



#### 字符串操作

##### join连接列表里的字符，前面的字符串作为间隔


```python
''.join(i for i in ['2','3','4'])
```




    '234'




```python
','.join(i for i in ['2','3','4'])
```




    '2,3,4'



##### str.startswith() 和 str.endswith() 


```python
filename = 'spam.txt'
filename.endswith('.txt')
```




    True



查询多个


```python
filename.endswith(('.txt','.txt1'))
```




    True



##### re正则

+ 使用同一个模式去做多次匹配，将模式字符串预编译为模式对象
+ match只会匹配字符串的开头部分，后面不检查，使用group获取值
+ search匹配字符串的所有部分
+ findall返回所有的匹配，若有捕获分组，则只返回捕获，且捕获以分组返回
+ finditer使用迭代器返回所有的匹配，且包含捕获内容
+ re.sub(r'(\d+)/(\d+)/(\d+)', r'\3-\1-\2', text)替换内容


```python
import re
text1 = '11/27/2012'
datepat = re.compile(r'\d+/\d+/\d+')
datepat.match(text1).group()
```




    '11/27/2012'



用括号去捕获分组，使用group获取


```python
datepat = re.compile(r'(\d+)/(\d+)/(\d+)')
m = datepat.match('11/27/2012')
m.group(0),m.group(1),m.group(2)
```




    ('11/27/2012', '11', '27')




```python
import re
text1 = 'dsasda11/27/2012dasda hkl  12/22/202112'
datepat = re.compile(r'(\d+)/(\d+)/(\d+)')
datepat.search(text1).group(0)
```




    '11/27/2012'




```python
list(datepat.finditer(text1))[1].group(0)
```




    '12/22/202112'




```python
datepat.findall(text1)
```




    [('11', '27', '2012'), ('12', '22', '202112')]




```python
datepat = re.compile(r'[a-z]+(\d+)/(\d+)/(\d+)')
datepat.findall(text1)
```




    [('11', '27', '2012')]



使用命名分组


```python
text = 'Today is 11/27/2012. PyCon starts 3/13/2013.'
import re
re.sub(r'(?P<month>\d+)/(?P<day>\d+)/(?P<year>\d+)', r'\g<year>-\g<month>-\g<day>', text)
```




    'Today is 2012-11-27. PyCon starts 2013-3-13.'



##### 将Unicode文本标准化

在Unicode中，某些字符能够用多个合法的编码表示。


```python
s1 = 'Spicy Jalape\u00f1o'
s2 = 'Spicy Jalapen\u0303o'
```


```python
import unicodedata
t1 = unicodedata.normalize('NFC', s1)
t2 = unicodedata.normalize('NFC', s2)
t1 == t2

print(ascii(t1))
```

    'Spicy Jalape\xf1o'



```python
t3 = unicodedata.normalize('NFD', s1)
t4 = unicodedata.normalize('NFD', s2)
t3 == t4
```




    True



##### strip: 去掉文本字符串开头，结尾或者中间不想要的字符

+ 默认去掉空白字符
+ lstrip去掉左边的，rstrip去掉右边的
+ 也可以自行指定要去掉的字符串


```python
s = '   hello world \n'
s.strip(), s.lstrip()
```




    ('hello world', 'hello world \n')




```python
t = '-----hello====='
t.lstrip('-')
```




    'hello====='



##### 返回字符的ASCII数字


```python
ord('a')
```




    97



#### 字典操作

##### 创建字典

1. dict.fromkeys，默认值为none
2. dict(二维列表)
3. 通过**字典推导式**创建


```python
x = ('key1', 'key2', 'key3')
#y = 0

thisdict = dict.fromkeys(x)
thisdict
```




    {'key1': None, 'key2': None, 'key3': None}




```python
dict([['1',2],['3',4]])
```




    {'1': 2, '3': 4}

**字典推导式**：


```python
{i:2*i for i in range(3)}
```




    {0: 0, 1: 2, 2: 4}




```python
dict(zip('abc', [1, 2, 3]))
```




    {'a': 1, 'b': 2, 'c': 3}



##### 字典方法

+ keys   获取所有值
+ values  获取所有键
+ items  获取键值对对象
+ get(key,alt)  若不指定alt且无这个键，则返回None,否则返回alt
+ 使用['key']获取值时，若无键则报错
+ del dict['key']  删除键值对


```python
a = dict([['1',2],['3',4]])
type(a.get('2'))
```




    NoneType



##### 字典排序


```python
prices = {
    'ACME': 45.23,
    'AAPL': 612.78,
    'IBM': 205.55,
    'HPQ': 37.20,
    'FB': 10.75
}
min_price = min(zip(prices.values(), prices.keys()))
min_price
```




    (10.75, 'FB')




```python
sorted(zip(prices.values(), prices.keys()))
```




    [(10.75, 'FB'),
     (37.2, 'HPQ'),
     (45.23, 'ACME'),
     (205.55, 'IBM'),
     (612.78, 'AAPL')]




```python
sorted(prices,key=lambda x:x[1])
```




    ['AAPL', 'IBM', 'FB', 'ACME', 'HPQ']

##### 默认方法setdefault

**d.setdefault(k, [default]) ——可用于需要查找键+修改值的场景**

若字典里有键k，则直接返回k 所对应的值；若无，则让d[k] = default，然后返回default。

```python
my_dict.setdefault(key, []).append(new_value)
```

跟这样写：

```py
if key not in my_dict:
    my_dict[key] = []
    my_dict[key].append(new_value)
```

二者的效果是一样的，只不过后者至少要进行两次键查询——如果键不存在的话。

##### 以UserDict 为基类自定义映射类型

```python
import collections
class StrKeyDict(collections.UserDict): 
    def __missing__(self, key): 
        #  __missing__定义了键非字符串的处理方式
        if isinstance(key, str):
            raise KeyError(key)
        return self[str(key)]
    def __contains__(self, key):
        # 更简洁，不需要__missing__中的复杂处理
        return str(key) in self.data 
    def __setitem__(self, key, item):
        self.data[str(key)] = item 

```



#### 集合

#### 基本


```python
a = {1,2,3}
b = set([1,2,3])
a,b
```




    ({1, 2, 3}, {1, 2, 3})




```python
a.add(4)
a
```




    {1, 2, 3, 4}




```python
set(a)
```




    {'1', '2', '3'}



##### 集合运算

集（set）是由零个或多个不可修改的Python 数据对象组成的无序集合。集不允许重复元素，
并且写成由花括号包含、以逗号分隔的一系列值。空集由set()来表示。
![image.png](attachment:image.png)

![image.png](attachment:image.png)


```python
{1,2,3}&{2,3,4}
```




    {2, 3}




```python
a = {"1":1, "2":2, "3":3}
b = {"2":2, "3":3, "4":4}

c = set(a)&set(b)
c
```




    {'2', '3'}



#### 浅拷贝（非递归）和深拷贝（递归）

+ 浅拷贝只会拷贝到第一层的数据，对于数组，若使用a = list(ls)来赋值，则只有一维数组有效，二维数组则只会有引用
+ 深拷贝会循环遍历到所有的数据，因此不存在共享指针的问题
+ 使用copy模块的deepcopy来深拷贝


```python
a = [1,[1,2]]
b = list(a)
b[1][0] = 12
a,b
```




    ([1, [12, 2]], [1, [12, 2]])




```python
import copy
a = [1,[1,2]]
b = copy.deepcopy(a)
b[1][0] = 12
a,b
```




    ([1, [1, 2]], [1, [12, 2]])



##### 使用\*对字典和列表浅拷贝


```python
a = {"x": 55, "y": 66,"z":[1,2,3]}
b = {"a": "foo", "b": "bar"}
c = {**a, **b}
```


```python
a['x'] =10
c['x'] =20
a['z'][0] = 123
c['z'][0] = 123
a,c
```




    ({'x': 10, 'y': 66, 'z': [123, 2, 3]},
     {'x': 20, 'y': 66, 'z': [123, 2, 3], 'a': 'foo', 'b': 'bar'})



#### itertools和collections模块的常用方法

##### itertools.groupby(s)分组


```python
import itertools
s = "AAABBCCCCC"
for k, v in itertools.groupby(s):
    print(k, list(v))
```

    A ['A', 'A', 'A']
    B ['B', 'B']
    C ['C', 'C', 'C', 'C', 'C']



```python
import itertools
s = [[1,2,3],[3,4,3],[0,0,0],[6,3,2]]
# 以是否大于10分组
for k, v in itertools.groupby(s,key=lambda nums:sum(nums)>10):
    print(k, list(v))
```

    False [[1, 2, 3], [3, 4, 3], [0, 0, 0]]
    True [[6, 3, 2]]


##### 分类计数collections.Counter


```python
import collections
s = "AAABBCCCCC"
# 以是否大于10分组
collections.Counter(s)
```




    Counter({'A': 3, 'B': 2, 'C': 5})



### 数据结构

#### collections.namedtuple：具名元组

可以用来构建一个带字段名的元组和一个有名字的类

```py
>>> from collections import namedtuple
>>> City = namedtuple('City', 'name country population coordinates') 
>>> tokyo = City('Tokyo', 'JP', 36.933, (35.689722, 139.691667)) 
>>> tokyo
City(name='Tokyo', country='JP', population=36.933, coordinates=(35.689722,
139.691667))
>>> tokyo.population 
36.933
>>> tokyo.coordinates
(35.689722, 139.691667)
>>> tokyo[1]
'JP'
>>> City._fields
('name', 'country', 'population', 'coordinates')
>>> delhi._asdict()
OrderedDict([('name', 'Delhi NCR'), ('country', 'IN'), ('population',
21.935), ('coordinates', LatLong(lat=28.613889, long=77.208889))])
```



#### collections.deque：双端队列

deque是双端队列（double-ended queue）的缩写，由于两端都能编辑，deque既可以用来实现栈（stack）也可以用来实现队列（queue）。相比于list实现的队列，deque实现拥有更低的时间和空间复杂度。list实现在出队（pop）和插入（insert）时的空间复杂度大约为O(n)，deque在出队（pop）和入队（append）时的时间复杂度是O(1)。
迭代操作或者其他操作的时候，可用于保留最后有限几个元素


```python
from collections import deque
q = deque(maxlen=3)  #指定最大长度
q.append(1)
q.append(3)
q.append(4)
q.append(2)
q
```




    deque([3, 4, 2])



使用deque来存储序列最后几个元素而不占用额外空间


```python
from collections import deque
def tail(path, n=4):
    f = path
    return deque(f, n)

tail("/etc/hosts")
```




    deque(['o', 's', 't', 's'])



#### heapq: 查找最大或最小的 N 个元素

当要查找的元素个数相对比较小的时候，函数 nlargest() 和 nsmallest() 是很合适的。 如果你仅仅想查找唯一的最小或最大（N=1）的元素的话，那么使用 min() 和 max() 函数会更快些。 类似的，如果 N 的大小和集合大小接近的时候，通常先排序这个集合然后再使用切片操作会更快点 （ sorted(items)[:N] 或者是 sorted(items)[-N:] ）。 需要在正确场合使用函数 nlargest() 和 nsmallest() 才能发挥它们的优势 （如果 N 快接近集合大小了，那么使用排序操作会更好些）。


```python
import heapq
portfolio = [
    {'name': 'IBM', 'shares': 100, 'price': 91.1},
    {'name': 'AAPL', 'shares': 50, 'price': 543.22},
    {'name': 'FB', 'shares': 200, 'price': 21.09},
    {'name': 'HPQ', 'shares': 35, 'price': 31.75},
    {'name': 'YHOO', 'shares': 45, 'price': 16.35},
    {'name': 'ACME', 'shares': 75, 'price': 115.65}
]
cheap = heapq.nsmallest(3, portfolio, key=lambda s: s['price'])
expensive = heapq.nlargest(3, portfolio, key=lambda s: s['price'])
```

##### 使用堆排序原理：

堆数据结构最重要的特征是 heap[0] 永远是最小的元素


```python
nums = [1, 8, 2, 23, 7, -4, 18, 23, 42, 37, 2]
heapq.heapify(nums)
nums
```




    [-4, 2, 1, 23, 7, 2, 18, 23, 42, 37, 8]




```python
heapq.heappop(nums)
```




    18




```python
heapq.heappop(nums)
nums
```




    [1, 2, 2, 23, 7, 8, 18, 23, 42, 37]



##### 实现优先级队列

index 变量的作用是保证同等优先级元素的正确排序。 通过保存一个不断增加的 index 下标变量，可以确保元素按照它们插入的顺序排序。 而且， index 变量也在相同优先级元素比较的时候不会报错。


```python
import heapq

class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def push(self, item, priority):
        heapq.heappush(self._queue, (-priority, self._index, item))
        self._index += 1

    def pop(self):
        return heapq.heappop(self._queue)[-1]
```


```python
>>> class Item:
...     def __init__(self, name):
...         self.name = name
...     def __repr__(self):
...         return 'Item({!r})'.format(self.name)
...
>>> q = PriorityQueue()
>>> q.push(Item('foo'), 1)
>>> q.push(Item('bar'), 5)
>>> q.push(Item('spam'), 4)
>>> q.push(Item('grok'), 1)
>>> q.pop()
Item('bar')
>>> q.pop()
Item('spam')
>>> q.pop()
Item('foo')
>>> q.pop()
Item('grok')
>>>
```



### 工具库

#### Redis

在Python中使用Redis是一个相对简单的过程，主要通过安装Redis服务器和使用Python的Redis客户端库来实现。以下是使用Redis的一般步骤：

1. **安装Redis服务器**：
   
   - 首先，你需要在你的机器上安装Redis服务器。你可以从Redis官网下载并安装它，或者使用包管理器安装（例如，在Ubuntu上使用`sudo apt-get install redis-server`）。
   
2. **安装Python的Redis客户端库**：
   
   - 你可以使用pip来安装Redis的Python客户端库，最常用的是`redis-py`。通过运行`pip install redis`来安装。
   
3. **连接到Redis服务器**：
   
   - 使用`redis-py`库，你可以创建一个连接到Redis服务器的实例。
   
   ```python
   import redis
   
   # 创建连接对象
   r = redis.Redis(host='localhost', port=6379, db=0)
   ```
   
4. **基本操作**：
   - 一旦连接到Redis，你可以执行各种基本操作，如设置键值对、获取值、删除键等。

   ```python
   # 设置键值对
   r.set('my_key', 'my_value')
   
   # 获取键的值
   value = r.get('my_key')
   print(value)  # 输出: b'my_value'
   
   # 删除键
   r.delete('my_key')
   ```

5. **使用哈希**：
   - Redis支持哈希类型，可以存储键值对的集合。

   ```python
   # 设置哈希
   r.hset('my_hash', 'field1', 'value1')
   r.hset('my_hash', 'field2', 'value2')
   
   # 获取哈希中的字段值
   value = r.hget('my_hash', 'field1')
   print(value)  # 输出: b'value1'
   
   # 获取哈希中的所有字段和值
   all_values = r.hgetall('my_hash')
   print(all_values)  # 输出: {'field1': b'value1', 'field2': b'value2'}
   ```

6. **使用列表**：
   - Redis的列表是简单的字符串列表，按照插入顺序排序。

   ```python
   # 将元素添加到列表
   r.lpush('my_list', 'item1')
   r.lpush('my_list', 'item2')
   
   # 获取列表中的元素
   value = r.lindex('my_list', 0)  # 获取列表的第一个元素
   print(value)  # 输出: b'item2'
   
   # 获取列表的切片
   values = r.lrange('my_list', 0, -1)  # 获取列表的所有元素
   print(values)  # 输出: [b'item2', b'item1']
   ```

7. **使用集合**：
   - Redis的集合是无序的，并且集合中的元素是唯一的。

   ```python
   # 将元素添加到集合
   r.sadd('my_set', 'item1')
   r.sadd('my_set', 'item2')
   
   # 获取集合中的元素
   values = r.smembers('my_set')
   print(values)  # 输出可能是: {'item1', 'item2'}
   ```

8. **使用有序集合**：
   - 有序集合类似于集合，但它可以为每个元素关联一个分数，Redis会根据这个分数为元素进行排序。

   ```python
   # 添加元素到有序集合并设置分数
   r.zadd('my_zset', {'item1': 1, 'item2': 2})
   
   # 获取有序集合中的元素
   values = r.zrange('my_zset', 0, -1, withscores=True)
   print(values)  # 输出可能是: [(b'item1', 1), (b'item2', 2)]
   ```

9. **发布/订阅**：
   - Redis支持发布订阅模式，允许你发送消息到频道，并让订阅这些频道的客户端接收消息。

   ```python
   # 发布消息
   r.publish('my_channel', 'Hello world!')
   
   # 订阅频道
   pubsub = r.pubsub()
   pubsub.subscribe(**{'my_channel': lambda message: print(message['data'])})
   pubsub.run_in_thread(sleep_time=0.001)
   ```

10. **连接池**：
    
    - 为了提高性能，可以使用连接池来管理Redis连接。
    
    ```python
    from redis import ConnectionPool
    
    pool = ConnectionPool(host='localhost', port=6379, db=0)
    r = redis.Redis(connection_pool=pool)
    ```
    
11. **使用SSL连接**：
    - 如果你的Redis服务器配置了SSL，你可以使用SSL连接。

    ```python
    r = redis.Redis(host='localhost', port=6379, ssl=True, ssl_ca_certs='ca.crt', ssl_cert_reqs='required')
    ```

12. **异常处理**：
    - 使用Redis时，你可能会遇到各种异常，如连接错误、数据错误等。使用try-except块来处理这些异常。

    ```python
    try:
        r.set('my_key', 'my_value')
    except redis.RedisError as e:
        print(f'An error occurred: {e}')
    ```

这些是使用Python和Redis进行基本操作的一些示例。`redis-py`库提供了丰富的API来支持Redis的所有功能，包括管道、事务、Lua脚本执行等高级特性。你可以查阅[`redis-py`官方文档](https://redis-py.readthedocs.io/en/latest/)来获取更详细的信息和高级用法。

#### RabbitMQ

Python 中使用 RabbitMQ 的一个简单例子可以包括以下几个步骤：

1. **安装 RabbitMQ 服务器**：首先，你需要在你的机器上安装 RabbitMQ 服务器。可以从 RabbitMQ 官网下载并安装。

2. **安装 Python 客户端库**：使用 pip 安装 `pika`，这是 Python 的 RabbitMQ 客户端库。

   ```bash
   pip install pika
   ```

3. **创建 RabbitMQ 连接**：在 Python 脚本中，首先需要创建一个到 RabbitMQ 服务器的连接。

   ```python
   import pika
   
   connection = pika.BlockingConnection(
       pika.ConnectionParameters(host = MQ_CONFIG.get('host'),port = MQ_CONFIG.get('port'),
       virtual_host = MQ_CONFIG.get('vhost'),credentials = credentials, heartbeat=int(1 * 60 * 60)))
   channel = connection.channel()
   # basic_qos 方法的 prefetch_count 参数指定了消费者在处理完当前消息之前可以接收的消息数量。
   # 如果设置为 1，这意味着消费者一次只能接收一个消息，并且必须在接收下一个消息之前确认当前消息。
   # 与MQ服务端无关
   channel.basic_qos(prefetch_count=1)
   ```

4. **声明队列**：声明一个队列，消息将发送到这个队列。

   ```python
   channel.queue_declare(queue='hello')
   ```

5. **发送消息**：创建一个发送者，将消息发送到队列。

   ```python
   channel.basic_publish(exchange='',
                         routing_key='hello',
                         body='Hello World!')
   print(" [x] Sent 'Hello World!'")
   ```

6. **接收消息**：创建一个接收者，监听队列并接收消息。

   ```python
   def callback(ch, method, properties, body):
       print(" [x] Received %r" % body)

   channel.basic_consume(queue='hello',
                          on_message_callback=callback,
                          auto_ack=True)

   print(' [*] Waiting for messages. To exit press CTRL+C')
   channel.start_consuming()
   ```

7. **关闭连接**：在消息接收完毕后，关闭连接。

   ```python
   connection.close()
   ```



### 设计模式

模板方法模式（Template Method Pattern）和建造者模式（Builder Pattern）是两种不同的设计模式，它们在软件设计中有不同的应用场景和目的。

**模板方法模式**

1. **定义**：模板方法模式是一种行为设计模式，它在超类中定义了一个算法的框架，同时允许子类在不改变算法结构的情况下重新定义算法的某些步骤。
2. **目的**：目的是让子类可以重新定义算法的某些步骤，而不必改变算法的整体结构。
3. **结构**：通常包含一个模板方法和一个或多个基本方法。模板方法是抽象的，而基本方法是具体实现或者留给子类实现。
4. **使用场景**：当你有一系列相似的行为，它们有固定的算法结构，但某些步骤可以在子类中有不同的实现时。

**建造者模式**

1. **定义**：建造者模式是一种创建型设计模式，用于创建一个复杂的对象，同时允许用户只通过指定复杂对象的类型和内容就能构建它们，隐藏了内部的构建细节。
2. **目的**：目的是将一个复杂对象的构建与其表示分离，使得同样的构建过程可以创建不同的表示。
3. **结构**：通常包含一个建造者接口、具体建造者实现、一个指挥者以及一个产品类。
4. **使用场景**：当你需要创建一个包含多个组成部分的复杂对象，同时希望将对象的构建和表示分离，或者当构建过程需要在不同的场景下有不同的变体时。

**主要区别**

- **目的不同**：模板方法模式关注于算法的结构和某些步骤的可定制性，而建造者模式关注于对象的构建过程和复杂对象的表示。
- **应用场景不同**：模板方法模式适用于有固定算法结构但需要在某些步骤上提供灵活性的情况；建造者模式适用于创建具有复杂内部表示的对象，且构建过程可能因场景而异。
- **结构不同**：模板方法模式通常包含模板方法和基本方法，而建造者模式包含建造者接口、具体建造者、指挥者和产品类。
- **灵活性**：模板方法模式的灵活性在于算法的某些步骤可以被重新定义，而建造者模式的灵活性在于可以构建出不同类型的复杂对象。

两种模式在实际应用中可以根据具体的需求和场景来选择使用。

```python
from abc import ABC, abstractmethod

# 模板方法模式：游戏单位创建流程
class UnitFactory(ABC):
    @abstractmethod
    def build_structure(self):
        pass

    @abstractmethod
    def add_capabilities(self):
        pass

    @abstractmethod
    def finalize_unit(self):
        pass

    def create_unit(self):
        self.build_structure()
        self.add_capabilities()
        self.finalize_unit()
        return self.unit

# 建造者模式：实现不同类型的单位建造者
class WarriorUnitBuilder(UnitFactory):
    def __init__(self):
        self.unit = Warrior()

    def build_structure(self):
        print("Building structure for warrior unit.")
        self.unit.set_armor()

    def add_capabilities(self):
        print("Adding capabilities to warrior unit.")
        self.unit.set_weapon()

    def finalize_unit(self):
        print("Finalizing warrior unit.")
        self.unit.activate()

class MageUnitBuilder(UnitFactory):
    def __init__(self):
        self.unit = Mage()

    def build_structure(self):
        print("Building structure for mage unit.")
        self.unit.set_robes()

    def add_capabilities(self):
        print("Adding capabilities to mage unit.")
        self.unit.learn_spells()

    def finalize_unit(self):
        print("Finalizing mage unit.")
        self.unit.meditate()

# 使用模板方法与建造者模式
warrior_builder = WarriorUnitBuilder()
warrior = warrior_builder.create_unit()

mage_builder = MageUnitBuilder()
mage = mage_builder.create_unit()
```
