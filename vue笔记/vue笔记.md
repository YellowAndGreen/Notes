# vue.js

## Es6语法

变量声明：

var声明的变量没有区块的概念，可以跨块访问

let变量只能在区块作用域中访问，不能跨块访问

`async` 和 `await` 是 JavaScript 中用于处理异步操作的关键字。

首先，我们来看看什么是异步操作。在 JavaScript 中，有一些操作可能需要花费一些时间才能完成，例如网络请求或定时器。这些操作被称为异步操作，因为它们不会立即完成。

`async` 和 `await` 关键字让我们能够以更直观和易于理解的方式处理这些异步操作。

- `async`：这个关键字用于声明一个函数是异步的。异步函数总是返回一个 promise。如果函数的返回值不是一个 promise，那么 JavaScript 会自动将其包装成 promise。
- `await`：这个关键字只能在 `async` 函数中使用。它会暂停当前的异步函数，等待 promise 完成（即等待 promise 变为 resolved 或 rejected 状态），然后返回 promise 的结果值（如果 promise 是 resolved 的）或抛出错误（如果 promise 是 rejected 的）。

这是一个简单的例子：

```js
async function fetchUser(userId) {
  const response = await fetch(`https://api.example.com/users/${userId}`);
  const user = await response.json();
  return user;
}
```

在这个例子中，`fetchUser` 函数是一个异步函数，它使用 `await` 关键字等待 `fetch` 函数的结果。当 `fetch` 函数返回一个 promise 时，`await` 会暂停 `fetchUser` 函数的执行，直到 promise 完成。然后，它会继续执行 `fetchUser` 函数，并使用 promise 的结果值。

## 基础知识

Vue.config：全局配置

可通过`Vue.config.productionTip = false`阻止 vue 在启动时生成生产提示。

favicon.ico：网站图标，放在根目录下

### MVVM

1. M：模型(Model) ：对应data 中的数据
2. V：视图(View) ：模板
3. VM：视图模型(ViewModel) ： Vue 实例对象

因此一般用vm命名Vue 实例对象

![image-20211028211433269](img\image-20211028211433269.png)

​	5.data中所有的属性，最后都出现在了vm身上。

​        6.vm身上所有的属性 及 Vue原型上所有属性，在Vue模板中都可以直接使用。

### 创建Vue容器

`Vue.createApp()`方法返回vue的app实例，mount之后为根组件实例

子组件：

`app.component('botton-component',{...})`为app添加一个组件，第一个参数为组件名，第二个参数与根组件相同，该组件可直接在挂在的html容器中使用:`<botton-component></botton-component>`

这种方式为全局组件，必须在mount之前定义

局部组件是写在组件配置中的，只有该组件能使用。

```javascript
Vue.createApp({
    data() {
        return {
            msg: null,
            list:[]
        }
    },
    computed:{
        newdate(){
            if (this.list.length == 0) return null 
            return this.formatDate(this.list[this.list.length-1].time)
        }
    },
    components:{
        'child-component':child-component
    },
    methods:{
        submit(){
            if (this.msg) {
                this.list.push({
                    text:this.msg,
                    time: Date.now()
                })
                this.msg = null
            }
        },
        formatDate(value){
            let date = new Date(value);
            let year = date.getFullYear();
            let month = date.getMonth() + 1 +'';
            let day = date.getDate() +'';
            let hour = date.getHours() +'';
            let minute = date.getMinutes() +'';
            let second = date.getSeconds() +'';
            return `${year}-${month.padStart(2,'0')}-${day.padStart(2,'0')} ${hour.padStart(2,'0')}:${minute.padStart(2,'0')}:${second.padStart(2,'0')}`
        }
    }
}).mount("#app")
```



### 模版语法

#### 插值表达式

##### 文本插值

最基本的数据绑定形式是文本插值，它使用的是“Mustache”语法 (即双大括号)：

template

```
<span>Message: {{ msg }}</span>
```

双大括号标签会被替换为[相应组件实例中](https://cn.vuejs.org/guide/essentials/reactivity-fundamentals.html#declaring-reactive-state) `msg` 属性的值。同时每次 `msg` 属性更改时它也会同步更新。

##### 使用 JavaScript 表达式

至此，我们仅在模板中绑定了一些简单的属性名。但是 Vue 实际上在所有的数据绑定中都支持完整的 JavaScript 表达式：

template

```
{{ number + 1 }}

{{ ok ? 'YES' : 'NO' }}

{{ message.split('').reverse().join('') }}

<div :id="`list-${id}`"></div>
```

这些表达式都会被作为 JavaScript ，以当前组件实例为作用域解析执行。

在 Vue 模板内，JavaScript 表达式可以被使用在如下场景上：

- 在文本插值中 (双大括号)
- 在任何 Vue 指令 (以 `v-` 开头的特殊 attribute) attribute 的值中

##### 单一表达式

每个绑定仅支持**单一表达式**，也就是一段能够被求值的 JavaScript 代码。一个简单的判断方法是是否可以合法地写在 `return` 后面。

因此，下面的例子都是**无效**的：

template

```
<!-- 这是一个语句，而非表达式 -->
{{ var a = 1 }}

<!-- 条件控制也不支持，请使用三元表达式 -->
{{ if (ok) { return message } }}
```

##### 调用函数

可以在绑定的表达式中使用一个组件暴露的方法：

template

```
<time :title="toTitleDate(date)" :datetime="date">
  {{ formatDate(date) }}
</time>
```

TIP：绑定在表达式中的方法在组件每次更新时都会被重新调用，因此**不**应该产生任何副作用，比如改变数据或触发异步操作。

### [内置指令](https://cn.vuejs.org/api/built-in-directives.html)

指令是带有 `v-` 前缀的特殊 attribute。Vue 提供了许多[内置指令](https://cn.vuejs.org/api/built-in-directives.html)，包括上面我们所介绍的 `v-bind` 和 `v-html`。

指令 attribute 的期望值为一个 JavaScript 表达式 (除了少数几个例外，即之后要讨论到的 `v-for`、`v-on` 和 `v-slot`)。一个指令的任务是在其表达式的值变化时响应式地更新 DOM。

1. 绑定标签属性v-bind（缩写为:）

   `<span v-bind:title="message">   11  </span>`

   <mark>v-bind是单向绑定，数据只能从data中传入，而不能从dom中传出</mark>

   若为布尔属性且不存在，则该标签title不会渲染在span中

2. 绑定表单类元素v-model

   `<input type="text" v-model:value="name">`

   缩写为：

   `<input type="text" v-model="name"><br/>`

   <mark>v-model是双向绑定，dom和data中的数据同步</mark>

3.  [`v-if`](https://cn.vuejs.org/api/built-in-directives.html#v-if)和`v-show`

   ```
   <p v-if="seen">Now you see me</p>
   ```

   这里，`v-if` 指令会基于表达式 `seen` 的值的真假来移除/插入该 `<p>` 元素，该元素不会被渲染。

   `v-show`的元素会被渲染，但只是隐藏该元素。

4. `v-for`

​	基于原始数据多次渲染元素或模板块。

​	**期望的绑定值类型：**`Array | Object | number | string | Iterable`

​	**详细信息**

​	指令值必须使用特殊语法 `alias in expression` 为正在迭代的元素提供一个别名：

```
<div v-for="item in items">
  {{ item.text }}
</div>
```

​	或者，你也可以为索引指定别名 (如果用在对象，则是键值)：

```
<div v-for="(item, index) in items"></div>
<div v-for="(value, key) in object"></div>
<div v-for="(value, name, index) in object"></div>
```

`	v-for` 的默认方式是尝试就地更新元素而不移动它们。要强制其重新排序元素，你需要用特殊 attribute `key` 来提供一个排序提示：

```
<div v-for="item in items" :key="item.id">
  {{ item.text }}
</div>
```

​	`v-for` 也可以用于 [Iterable Protocol](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Iteration_protocols#The_iterable_protocol) 的实现，包括原生 `Map` 和 `Set`。

5. 使用v-on:xxx 或 @xxx 绑定事件（缩写为@）

​              1.事件的回调需要配置在methods对象中，最终会在vm上；

​              2.methods中配置的函数，不要用箭头函数！否则this就不是vm了；

​              3.methods中配置的函数，都是被Vue所管理的函数，this的指向是vm 或 组件实例对象；

​              4.@click="demo" 和 @click="demo($event)" 效果一致，但后者可以传参；

```html
		<div id="root">
			<h2>欢迎来到{{name}}学习</h2>
			<!-- <button v-on:click="showInfo">点我提示信息</button> -->
			<button @click="showInfo1">点我提示信息1（不传参）</button>
			<button @click="showInfo2($event,66)">点我提示信息2（传参）</button>
		</div>
```

```vue
		const vm = new Vue({
			el:'#root',
			data:{
				name:'尚硅谷',
			},
			methods:{
				showInfo1(event){
					// console.log(event.target.innerText)
					// console.log(this) //此处的this是vm
					alert('同学你好！')
				},
				showInfo2(event,number){
					console.log(event,number)
					// console.log(event.target.innerText)
					// console.log(this) //此处的this是vm
					alert('同学你好！！')
				}
			}
		})
```



### 属性

#### data

data属性有两种写法：

```vue
new Vue({
	el:'#root',
	//data的第一种写法：对象式
	/* data:{
		name:'尚硅谷'
	} */

	//data的第二种写法：函数式
	data(){
		console.log('@@@',this) //此处的this是Vue实例对象
		return{
			name:'尚硅谷'
		}
	}
})
```

Vue会通过响应式系统将data包裹起来，以`$data`存储在组件示例中。注意，在挂载后调用组件添加的变量是没有响应输赢的。

```vue
const vm=app.mount('#app')

console.log(vm.$data.count)
console.log(vm.count)
```

#### 计算属性

推荐使用**计算属性**来描述依赖响应式状态的复杂逻辑

```vue
<script setup>
import { reactive, computed } from 'vue'

const author = reactive({
  name: 'John Doe',
  books: [
    'Vue 2 - Advanced Guide',
    'Vue 3 - Basic Guide',
    'Vue 4 - The Mystery'
  ]
})

// 一个计算属性 ref
const publishedBooksMessage = computed(() => {
  return author.books.length > 0 ? 'Yes' : 'No'
})
</script>

<template>
  <p>Has published books:</p>
  <span>{{ publishedBooksMessage }}</span>
</template>
```

#### 侦听器

计算属性允许我们声明性地计算衍生值。然而在有些情况下，我们需要在状态变化时执行一些“副作用”：例如更改 DOM，或是根据异步操作的结果去修改另一处的状态。

```vue
<script setup>
import { ref, watch } from 'vue'

const question = ref('')
const answer = ref('Questions usually contain a question mark. ;-)')

// 可以直接侦听一个 ref
watch(question, async (newQuestion, oldQuestion) => {
  if (newQuestion.indexOf('?') > -1) {
    answer.value = 'Thinking...'
    try {
      const res = await fetch('https://yesno.wtf/api')
      answer.value = (await res.json()).answer
    } catch (error) {
      answer.value = 'Error! Could not reach the API. ' + error
    }
  }
})
</script>

<template>
  <p>
    Ask a yes/no question:
    <input v-model="question" />
  </p>
  <p>{{ answer }}</p>
</template>
```

**注意，你不能直接侦听响应式对象的属性值**，例如:

```js
const obj = reactive({ count: 0 })

// 错误，因为 watch() 得到的参数是一个 number
watch(obj.count, (count) => {
  console.log(`count is: ${count}`)
})
```

这里需要用一个返回该属性的 getter 函数：

```js
// 提供一个 getter 函数
watch(
  () => obj.count,
  (count) => {
    console.log(`count is: ${count}`)
  }
)
```

#### 深层侦听器

直接给 `watch()` 传入一个响应式对象，会隐式地创建一个深层侦听器——该回调函数在所有嵌套的变更时都会被触发：

```js
const obj = reactive({ count: 0 })

watch(obj, (newValue, oldValue) => {
  // 在嵌套的属性变更时触发
  // 注意：`newValue` 此处和 `oldValue` 是相等的
  // 因为它们是同一个对象！
})

obj.count++
```

相比之下，一个返回响应式对象的 getter 函数，只有在返回不同的对象时，才会触发回调：

```js
watch(
  () => state.someObject,
  () => {
    // 仅当 state.someObject 被替换时触发
  }
)
```

你也可以给上面这个例子显式地加上 `deep` 选项，强制转成深层侦听器：

```js
watch(
  () => state.someObject,
  (newValue, oldValue) => {
    // 注意：`newValue` 此处和 `oldValue` 是相等的
    // *除非* state.someObject 被整个替换了
  },
  { deep: true }
)
```

#### 即时回调的侦听器

`watch` 默认是懒执行的：仅当数据源变化时，才会执行回调。但在某些场景中，我们希望在创建侦听器时，立即执行一遍回调。举例来说，我们想请求一些初始数据，然后在相关状态更改时重新请求数据。

我们可以通过传入 `immediate: true` 选项来强制侦听器的回调立即执行：

```js
watch(source, (newValue, oldValue) => {
  // 立即执行，且当 `source` 改变时再次执行
}, { immediate: true })
```

#### `watchEffect`

侦听器的回调使用与源完全相同的响应式状态是很常见的。例如下面的代码，在每当 `todoId` 的引用发生变化时使用侦听器来加载一个远程资源：

```js
const todoId = ref(1)
const data = ref(null)

watch(todoId, async () => {
  const response = await fetch(
    `https://jsonplaceholder.typicode.com/todos/${todoId.value}`
  )
  data.value = await response.json()
}, { immediate: true })
```

特别是注意侦听器是如何两次使用 `todoId` 的，一次是作为源，另一次是在回调中。

我们可以用 [`watchEffect` 函数](https://cn.vuejs.org/api/reactivity-core.html#watcheffect) 来简化上面的代码。`watchEffect()` 允许我们自动跟踪回调的响应式依赖。上面的侦听器可以重写为：

```js
watchEffect(async () => {
  const response = await fetch(
    `https://jsonplaceholder.typicode.com/todos/${todoId.value}`
  )
  data.value = await response.json()
})
```

这个例子中，回调会立即执行，不需要指定 `immediate: true`。在执行期间，它会自动追踪 `todoId.value` 作为依赖（和计算属性类似）。每当 `todoId.value` 变化时，回调会再次执行。有了 `watchEffect()`，我们不再需要明确传递 `todoId` 作为源值。

你可以参考一下[这个例子](https://cn.vuejs.org/examples/#fetching-data)的 `watchEffect` 和响应式的数据请求的操作。

对于这种只有一个依赖项的例子来说，`watchEffect()` 的好处相对较小。但是对于有多个依赖项的侦听器来说，使用 `watchEffect()` 可以消除手动维护依赖列表的负担。此外，如果你需要侦听一个嵌套数据结构中的几个属性，`watchEffect()` 可能会比深度侦听器更有效，因为它将只跟踪回调中被使用到的属性，而不是递归地跟踪所有的属性。



### 数据代理

<mark>数据代理：通过一个对象代理对另一个对象中属性的操作（读/写）</mark>

#### Object.defineProperty方法

```
let number = 18
let person = {
	name:'张三',
	sex:'男',
}

Object.defineProperty(person,'age',{
	// value:18,
	// enumerable:true, //控制属性是否可以枚举，默认值是false
	// writable:true, //控制属性是否可以被修改，默认值是false
	// configurable:true //控制属性是否可以被删除，默认值是false

	//当有人读取person的age属性时，get函数(getter)就会被调用，且返回值就是age的值
	get(){
		console.log('有人读取age属性了')
		return number
	},

	//当有人修改person的age属性时，set函数(setter)就会被调用，且会收到修改的具体值
	set(value){
		console.log('有人修改了age属性，且值是',value)
		number = value
	}

})
```



#### Vue中的数据代理

通过Object.defineProperty方法来实现，因此获取数据不需要{{_data.name}}而是{{name}}

只有data里的数据才会进行数据代理

![image-20211028214313265](img\image-20211028214313265.png)

## 常用组合 API

### 1. setup

1. 理解：Vue3.0中一个新的配置项，值为一个函数。
2. setup是所有<strong style="color:#DD5145">Composition API（组合API）</strong><i style="color:gray;font-weight:bold">“ 表演的舞台 ”</i>。
3. 组件中所用到的：数据、方法等等，均要配置在setup中。
4. setup函数的两种返回值：
   1. 若返回一个对象，则对象中的属性、方法, 在模板中均可以直接使用。（重点关注！）
   2. <span style="color:#aad">若返回一个渲染函数：则可以自定义渲染内容。（了解）</span>
5. 注意点：
   1. 尽量不要与Vue2.x配置混用
      - Vue2.x配置（data、methos、computed...）中<strong style="color:#DD5145">可以访问到</strong>setup中的属性、方法。
      - 但在setup中<strong style="color:#DD5145">不能访问到</strong>Vue2.x配置（data、methos、computed...）。
      - 如果有重名, setup优先。
   2. setup不能是一个async函数，因为返回值不再是return的对象, 而是promise, 模板看不到return对象中的属性。（后期也可以返回一个Promise实例，但需要Suspense和异步组件的配合）

###  2.ref函数

- 作用: 定义一个响应式的数据
- 语法: ```const xxx = ref(initValue)``` 
  - 创建一个包含响应式数据的<strong style="color:#DD5145">引用对象（reference对象，简称ref对象）</strong>。
  - JS中操作数据： ```xxx.value```
  - 模板中读取数据: 不需要.value，直接：```<div>{{xxx}}</div>```
- 备注：
  - 接收的数据可以是：基本类型、也可以是对象类型。
  - 基本类型的数据：响应式依然是靠``Object.defineProperty()``的```get```与```set```完成的。
  - 对象类型的数据：内部 <i style="color:gray;font-weight:bold">“ 求助 ”</i> 了Vue3.0中的一个新函数—— ```reactive```函数。

### 3.reactive函数

- 作用: 定义一个<strong style="color:#DD5145">对象类型</strong>的响应式数据（基本类型不要用它，要用```ref```函数）
- 语法：```const 代理对象= reactive(源对象)```接收一个对象（或数组），返回一个<strong style="color:#DD5145">代理对象（Proxy的实例对象，简称proxy对象）</strong>
- reactive定义的响应式数据是“深层次的”。
- 内部基于 ES6 的 Proxy 实现，通过代理对象操作源对象内部数据进行操作。



## Vue工程

### 创建方式

**1.使用 vue-cli 创建**

官方文档：https://cli.vuejs.org/zh/guide/creating-a-project.html#vue-create

```bash
## 查看@vue/cli版本，确保@vue/cli版本在4.5.0以上
vue --version
## 安装或者升级你的@vue/cli
npm install -g @vue/cli
## 创建
vue create vue_test
## 启动
cd vue_test
npm run serve
```

**2.使用 vite 创建**

官方文档：https://v3.cn.vuejs.org/guide/installation.html#vite

vite官网：https://vitejs.cn

- 什么是vite？—— 新一代前端构建工具。
- 优势如下：
  - 开发环境中，无需打包操作，可快速的冷启动。
  - 轻量快速的热重载（HMR）。
  - 真正的按需编译，不再等待整个应用编译完成。

```bash
## 创建工程
npm init vite-app <project-name>
## 进入工程目录
cd <project-name>
## 安装依赖
npm install
## 运行
npm run dev
```

### 单文件组件

Vue 的单文件组件 (即 `*.vue` 文件，英文 Single-File Component，简称 **SFC**) 是一种特殊的文件格式，使我们能够将一个 Vue 组件的模板、逻辑与样式封装在单个文件中。下面是一个单文件组件的示例：

```vue
<script setup>
import { ref } from 'vue'
const greeting = ref('Hello World!')
</script>

<template>
  <p class="greeting">{{ greeting }}</p>
</template>

<style>
.greeting {
  color: red;
  font-weight: bold;
}
</style>
```

如你所见，Vue 的单文件组件是网页开发中 HTML、CSS 和 JavaScript 三种语言经典组合的自然延伸。`<template>`、`<script>` 和 `<style>` 三个块在同一个文件中封装、组合了组件的视图、逻辑和样式。完整的语法定义可以查阅 [SFC 语法说明](https://cn.vuejs.org/api/sfc-spec.html)。

**单文件组件的导入和导出：**

```vue
import MyComponent from './MyComponent.vue'

export default {
  components: {
    MyComponent
  }
}
```



## 注意

1. js表达式 和 js代码(语句)的区别：

   1.表达式：<mark>**一个表达式会产生一个值**</mark>，可以放在任何一个需要值的地方：

   ​                  (1). a

   ​                  (2). a+b

   ​                  (3). demo(1)

   ​                  (4). x === y ? 'a' : 'b'

   

   2.js代码(语句)

   ​                  (1). if(){}

   ​                  (2). for(){}

## 其他
