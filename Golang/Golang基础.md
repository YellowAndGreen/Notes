# Golang

## 实现并发的工作池

```go
package main

import (
    "fmt"
    "time"
)

func worker(id int, jobs <-chan int, results chan<- int) {
    for j := range jobs {
        fmt.Println("worker", id, "started  job", j)
        time.Sleep(time.Second)
        fmt.Println("worker", id, "finished job", j)
        results <- j * 2
    }
}

func main() {

    const numJobs = 5
    jobs := make(chan int, numJobs)
    results := make(chan int, numJobs)

    for w := 1; w <= 3; w++ {
        go worker(w, jobs, results)
    }

    for j := 1; j <= numJobs; j++ {
        jobs <- j
    }
    close(jobs)
	//阻塞接收以避免主程序结束
    for a := 1; a <= numJobs; a++ {
        <-results
    }
}
```

## 使用WaitGroup来等待多协程结束

```go
package main

import (
    "fmt"
    "sync"
    "time"
)

func worker(id int) {
    fmt.Printf("Worker %d starting\n", id)

    time.Sleep(time.Second)
    fmt.Printf("Worker %d done\n", id)
}

func main() {

    var wg sync.WaitGroup

    for i := 1; i <= 5; i++ {
        wg.Add(1)
//避免在每个协程闭包中重复利用相同的 i 值
        i := i
//将 worker 调用包装在一个闭包中，可以确保通知 WaitGroup 此工作线程已完成。 这样，worker 线程本身就不必知道其执行中涉及的并发原语。
        go func() {
            defer wg.Done()
            worker(i)
        }()
    }
//阻塞，直到 WaitGroup 计数器恢复为 0； 即所有协程的工作都已经完成。
    wg.Wait()

}
```

## 排序

### 使用sort包

```go
package main

import (
    "fmt"
    "sort"
)

func main() {

    strs := []string{"c", "a", "b"}
    sort.Strings(strs)
    fmt.Println("Strings:", strs)

    ints := []int{7, 2, 4}
    sort.Ints(ints)
    fmt.Println("Ints:   ", ints)

    s := sort.IntsAreSorted(ints)
    fmt.Println("Sorted: ", s)
}
```

### 使用函数自定义排序

创建一个自定义类型， 为它实现 `sort.Interface` 接口的 `Len`、`Less` 和 `Swap` 方法， 然后对这个自定义类型的切片调用 `sort.Sort` 方法， 我们就可以通过任意函数对 Go 切片进行排序了。

```go
package main

import (
    "fmt"
    "sort"
)
//string[]类型的别名
type byLength []string

func (s byLength) Len() int {
    return len(s)
}
func (s byLength) Swap(i, j int) {
    s[i], s[j] = s[j], s[i]
}
func (s byLength) Less(i, j int) bool {
    return len(s[i]) < len(s[j])
}

func main() {
    fruits := []string{"peach", "banana", "kiwi"}
    sort.Sort(byLength(fruits))
    fmt.Println(fruits)
}
```



## 字符串

### strings

```go
package main

import (
	"fmt"
	s "strings"
)

var p = fmt.Println

func main() {

	p("Contains:  ", s.Contains("test", "es"))
	p("Count:     ", s.Count("test", "t"))
	p("HasPrefix: ", s.HasPrefix("test", "te"))
	p("HasSuffix: ", s.HasSuffix("test", "st"))
	p("Index:     ", s.Index("test", "e"))
	p("Join:      ", s.Join([]string{"a", "b"}, "-"))
	p("Repeat:    ", s.Repeat("a", 5))
	p("Replace:   ", s.Replace("foo", "o", "0", -1))
	p("Replace:   ", s.Replace("foo", "o", "0", 1))
	p("Split:     ", s.Split("a-b-c-d-e", "-"))
	p("ToLower:   ", s.ToLower("TEST"))
	p("ToUpper:   ", s.ToUpper("test"))
	p("Len: ", len("hello"))
	p("Char:", "hello"[1])
}

```

### regexp

```go
package main

import (
    "bytes"
    "fmt"
    "regexp"
)

func main() {
	// 返回是否找到
    match, _ := regexp.MatchString("p([a-z]+)ch", "peach")
    fmt.Println(match)

    r, _ := regexp.Compile("p([a-z]+)ch")

    fmt.Println(r.MatchString("peach"))
	// 返回匹配结果
    fmt.Println(r.FindString("peach punch"))
	// 返回匹配索引
    fmt.Println("idx:", r.FindStringIndex("peach punch"))
	// Submatch 返回完全匹配和局部匹配的字符串。 例如，这里会返回匹配 p([a-z]+)ch 和 ([a-z]+) 的信息。
    fmt.Println(r.FindStringSubmatch("peach punch"))
	
    fmt.Println(r.FindStringSubmatchIndex("peach punch"))
	// 带 All 的这些函数返回全部的匹配项
    fmt.Println(r.FindAllString("peach punch pinch", -1))

    fmt.Println("all:", r.FindAllStringSubmatchIndex(
        "peach punch pinch", -1))

    fmt.Println(r.FindAllString("peach punch pinch", 2))

    fmt.Println(r.Match([]byte("peach")))

    r = regexp.MustCompile("p([a-z]+)ch")
    fmt.Println("regexp:", r)

    fmt.Println(r.ReplaceAllString("a peach", "<fruit>"))

    in := []byte("a peach")
    out := r.ReplaceAllFunc(in, bytes.ToUpper)
    fmt.Println(string(out))
}
```



## 时间

1. 可使用time.Minute作为时间块传入，比如time.sleep和time.Sub中

### 基本

```go
package main

import (
    "fmt"
    "time"
)

func main() {
    p := fmt.Println

    now := time.Now()
    p(now)
	//创建时间对象
    then := time.Date(
        2009, 11, 17, 20, 34, 58, 651387237, time.UTC)
    p(then)

    p(then.Year())
    p(then.Month())
    p(then.Day())
    p(then.Hour())
    p(then.Minute())
    p(then.Second())
    p(then.Nanosecond())
    p(then.Location())

    p(then.Weekday())
	//时间比较操作
    p(then.Before(now))
    p(then.After(now))
    p(then.Equal(now))
	//时间加减法
    diff := now.Sub(then)
    p(diff)

    p(diff.Hours())
    p(diff.Minutes())
    p(diff.Seconds())
    p(diff.Nanoseconds())
	//时间加法，只能为时区对象
    p(then.Add(diff))
    p(then.Add(-diff))
}
```

### 时间的格式化和解析

1. 时间对象的字符串格式化：`t.Format(time.RFC3339)`

2. 将字符串解析为时间对象：

   ```go
   time.Parse(time.RFC3339,"2012-11-01T22:08:41+00:00")
   ```

3. 以指定的格式进行格式化和解析

   ```go
   t = time.Now()
   
   p(t.Format("3:04PM"))
   p(t.Format("Mon Jan _2 15:04:05 2006"))
   p(t.Format("2006-01-02T15:04:05.999999-07:00"))
   form := "3 04 PM"
   t2, e := time.Parse(form, "8 41 PM")
   ```



## 数字

### 随机数

1. 使用`rand.Intn(100)`生成0-100的随机整数
2. 使用`rand.Float64()`生成0-1的随机浮点数
3. 指定随机数种子：

```go
s1 := rand.NewSource(time.Now().UnixNano())
r1 := rand.New(s1)
r1.Intn(100)
```



### 字符串转数字

1. 第二个参数是是否自动识别是什么进制，第三个参数是数字bits

```go
package main

import (
    "fmt"
    "strconv"
)

func main() {

    f, _ := strconv.ParseFloat("1.234", 64)
    fmt.Println(f)

    i, _ := strconv.ParseInt("123", 0, 64)
    fmt.Println(i)

    d, _ := strconv.ParseInt("0x1c8", 0, 64)
    fmt.Println(d)

    u, _ := strconv.ParseUint("789", 0, 64)
    fmt.Println(u)

    k, _ := strconv.Atoi("135")
    fmt.Println(k)

    _, e := strconv.Atoi("wat")
    fmt.Println(e)
}
```



## 文件

### 读文件

```go
package main

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
)

// 便捷检查错误的函数
func check(e error) {
	if e != nil {
		panic(e)
	}
}

func main() {
	filePath := "C:\\Users\\Administrator\\Desktop\\a.txt"
	// 打开文件并读入内存中
	dat, err := os.ReadFile(filePath)
	check(err)
	fmt.Print(string(dat))

	// 仅打开文件，还没读入
	f, err := os.Open(filePath)
	check(err)
    
    // 打开文件后，一个习惯性的操作是：立即使用 defer 调用文件的 Close。
	defer f.Close()
    
	// 读入文件，每次仅读入五个字节
	b1 := make([]byte, 5)
	n1, err := f.Read(b1)
	check(err)
	fmt.Printf("%d bytes: %s\n", n1, string(b1[:n1]))
	// 从第二个参数位置向右偏移第一个参数个字节
	o2, err := f.Seek(6, 0)
	check(err)
	fmt.Printf("Seek func:" + strconv.FormatInt(o2, 10) + "\n")

	// bufio 包实现了一个缓冲读取器，这可能有助于提高许多小读操作的效率
	r4 := bufio.NewReader(f)
	
	// 从第五个字节读取
	b4, err := r4.Peek(5)
	check(err)
	fmt.Printf("5 bytes: %s\n", string(b4))
}
```



### 写文件

```go
package main

import (
	"fmt"
	"os"
)

func check(e error) {
	if e != nil {
		panic(e)
	}
}

func main() {
	// 写入到一个文件
	d1 := []byte("hello\ngo\n")
	err := os.WriteFile("C:\\Users\\Administrator\\Desktop\\b.txt", d1, 0644)
	check(err)
	// 对于更细粒度的写入，先打开一个文件。
	f, err := os.Create("C:\\Users\\Administrator\\Desktop\\b.txt")
	check(err)

	// 打开文件后，一个习惯性的操作是：立即使用 defer 调用文件的 Close。
	defer f.Close()
	// 以字节写入并返回写入的字节数
	d2 := []byte{115, 111, 109, 101, 10}
	n2, err := f.Write(d2)
	check(err)
	fmt.Printf("wrote %d bytes\n", n2)

	// 写入字符串并返回写入的字节数
	n3, err := f.WriteString("writes\n")
	check(err)
	fmt.Printf("wrote %d bytes\n", n3)

	f.Sync()
	//
	//w := bufio.NewWriter(f)
	//n4, err := w.WriteString("buffered\n")
	//check(err)
	//fmt.Printf("wrote %d bytes\n", n4)
	//
	//w.Flush()

}
```

