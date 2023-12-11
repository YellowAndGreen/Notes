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



### 响应式基础

#### `ref()`

在组合式 API 中，推荐使用 [`ref()`](https://cn.vuejs.org/api/reactivity-core.html#ref) 函数来声明响应式状态：

```vue
import { ref } from 'vue'

const count = ref(0)
```

`ref()` 接收参数，并将其包裹在一个带有 `.value` 属性的 ref 对象中返回：

```vue
const count = ref(0)

console.log(count) // { value: 0 }
console.log(count.value) // 0

count.value++
console.log(count.value) // 1
```

要在组件模板中访问 ref，请从组件的 `setup()` 函数中声明并返回它们：

```vue
import { ref } from 'vue'

export default {
  // `setup` 是一个特殊的钩子，专门用于组合式 API。
  setup() {
    const count = ref(0)

    // 将 ref 暴露给模板
    return {
      count
    }
  }
}
```

template:

```
<div>{{ count }}</div>
```

注意，在模板中使用 ref 时，我们**不**需要附加 `.value`。为了方便起见，当在模板中使用时，ref 会自动解包 (有一些[注意事项](https://cn.vuejs.org/guide/essentials/reactivity-fundamentals.html#caveat-when-unwrapping-in-templates))。

***深层响应性***

Ref 可以持有任何类型的值，包括深层嵌套的对象、数组或者 JavaScript 内置的数据结构，比如 `Map`。

Ref 会使它的值具有深层响应性。这意味着即使改变嵌套对象或数组时，变化也会被检测到

#### `<script setup>`

在 `setup()` 函数中手动暴露大量的状态和方法非常繁琐。幸运的是，我们可以通过使用[单文件组件 (SFC)](https://cn.vuejs.org/guide/scaling-up/sfc.html) 来避免这种情况。我们可以使用 `<script setup>` 来大幅度地简化代码：

```vue
<script setup>
import { ref } from 'vue'

const count = ref(0)

function increment() {
  count.value++
}
</script>

<template>
  <button @click="increment">
    {{ count }}
  </button>
</template>
```



<script setup> 中的顶层的导入、声明的变量和函数可在同一组件的模板中直接使用。你可以理解为模板是在同一作用域内声明的一个 JavaScript 函数——它自然可以访问与它一起声明的所有内容。

#### `reactive()`

还有另一种声明响应式状态的方式，即使用 `reactive()` API。与将内部值包装在特殊对象中的 ref 不同，`reactive()` 将使对象本身具有响应性：

```js
import { reactive } from 'vue'

const state = reactive({ count: 0 })
```

在模板中使用：

```js
<button @click="state.count++">
  {{ state.count }}
</button>
```

响应式对象是 [JavaScript 代理](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Proxy)，其行为就和普通对象一样。不同的是，Vue 能够拦截对响应式对象所有属性的访问和修改，以便进行依赖追踪和触发更新。

`reactive()` 将深层地转换对象：当访问嵌套对象时，它们也会被 `reactive()` 包装。当 ref 的值是一个对象时，`ref()` 也会在内部调用它。与浅层 ref 类似，这里也有一个 [`shallowReactive()`](https://cn.vuejs.org/api/reactivity-advanced.html#shallowreactive) API 可以选择退出深层响应性。

### 生命周期

![组件生命周期图示](./vue笔记.assets/lifecycle.16e4c08e.png)

每个 Vue 组件实例在创建时都需要经历一系列的初始化步骤，比如设置好数据侦听，编译模板，挂载实例到 DOM，以及在数据改变时更新 DOM。在此过程中，它也会运行被称为生命周期钩子的函数，让开发者有机会在特定阶段运行自己的代码。

**注册周期钩子**

举例来说，`onMounted` 钩子可以用来在组件完成初始渲染并创建 DOM 节点后运行代码：

```vue
<script setup>
import { onMounted } from 'vue'

onMounted(() => {
  console.log(`the component is now mounted.`)
})
</script>
```

### 组件基础

#### 使用组件

要使用一个子组件，我们需要在父组件中导入它。假设我们把计数器组件放在了一个叫做 `ButtonCounter.vue` 的文件中，这个组件将会以默认导出的形式被暴露给外部。

```
<script setup>
import ButtonCounter from './ButtonCounter.vue'
</script>

<template>
  <h1>Here is a child component!</h1>
  <ButtonCounter />
</template>
```

通过 `<script setup>`，导入的组件都在模板中直接可用。

当然，你也可以全局地注册一个组件，使得它在当前应用中的任何组件上都可以使用，而不需要额外再导入。关于组件的全局注册和局部注册两种方式的利弊，我们放在了[组件注册](https://cn.vuejs.org/guide/components/registration.html)这一章节中专门讨论。

组件可以被重用任意多次：

```
<h1>Here is a child component!</h1>
<ButtonCounter />
<ButtonCounter />
<ButtonCounter />
```

在单文件组件中，推荐为子组件使用 `PascalCase` 的标签名，以此来和原生的 HTML 元素作区分。虽然原生 HTML 标签名是不区分大小写的，但 Vue 单文件组件是可以在编译中区分大小写的。我们也可以使用 `/>` 来关闭一个标签。

如果你是**直接在 DOM 中**书写模板 (例如原生 `<template>` 元素的内容)，模板的编译需要遵从浏览器中 HTML 的解析行为。在这种情况下，你应该需要使用 `kebab-case` 形式并显式地关闭这些组件的标签。

```
<!-- 如果是在 DOM 中书写该模板 -->
<button-counter></button-counter>
<button-counter></button-counter>
<button-counter></button-counter>
```

#### 传递 props

**props是单向数据流，不能改变在子组件中改变props。**

如果我们正在构建一个博客，我们可能需要一个表示博客文章的组件。我们希望所有的博客文章分享相同的视觉布局，但有不同的内容。要实现这样的效果自然必须向组件中传递数据，例如每篇文章标题和内容，这就会使用到 props。

Props 是一种特别的 attributes，你可以在组件上声明注册。要传递给博客文章组件一个标题，我们必须在组件的 props 列表上声明它。这里要用到 [`defineProps`](https://cn.vuejs.org/api/sfc-script-setup.html#defineprops-defineemits) 宏：

```
<!-- BlogPost.vue -->
<script setup>
defineProps(['title'])
</script>

<template>
  <h4>{{ title }}</h4>
</template>
```

`defineProps` 是一个仅 `<script setup>` 中可用的编译宏命令，并不需要显式地导入。声明的 props 会自动暴露给模板。`defineProps` 会返回一个对象，其中包含了可以传递给组件的所有 props：

```
const props = defineProps(['title'])
console.log(props.title)
```

TypeScript 用户请参考：[为组件 props 标注类型](https://cn.vuejs.org/guide/typescript/composition-api.html#typing-component-props)

如果你没有使用 `<script setup>`，props 必须以 `props` 选项的方式声明，props 对象会作为 `setup()` 函数的第一个参数被传入：

```
export default {
  props: ['title'],
  setup(props) {
    console.log(props.title)
  }
}
```

一个组件可以有任意多的 props，默认情况下，所有 prop 都接受任意类型的值。

当一个 prop 被注册后，可以像这样以自定义 attribute 的形式传递数据给它：

template

```
<BlogPost title="My journey with Vue" />
<BlogPost title="Blogging with Vue" />
<BlogPost title="Why Vue is so fun" />
```

在实际应用中，我们可能在父组件中会有如下的一个博客文章数组：

js

```
const posts = ref([
  { id: 1, title: 'My journey with Vue' },
  { id: 2, title: 'Blogging with Vue' },
  { id: 3, title: 'Why Vue is so fun' }
])
```

这种情况下，我们可以使用 `v-for` 来渲染它们：

template

```
<BlogPost
  v-for="post in posts"
  :key="post.id"
  :title="post.title"
 />
```

#### 监听事件

子组件通过调用内置的 [**`$emit`** 方法](https://cn.vuejs.org/api/component-instance.html#emit)，通过传入事件名称来抛出一个事件，并通过 [`defineEmits`](https://cn.vuejs.org/api/sfc-script-setup.html#defineprops-defineemits) 宏来声明需要抛出的事件。在上级组件中声明被抛出的事件名，并确定处理方法`@enlarge-text="postFontSize += 0.1"`。

App.vue：

```vue
<script setup>
import { ref } from 'vue'
import BlogPost from './BlogPost.vue'
  
const posts = ref([
  { id: 1, title: 'My journey with Vue' },
  { id: 2, title: 'Blogging with Vue' },
  { id: 3, title: 'Why Vue is so fun' }
])

const postFontSize = ref(1)
</script>

<template>
	<div :style="{ fontSize: postFontSize + 'em' }">
    <BlogPost
      v-for="post in posts"
      :key="post.id"
      :title="post.title"
      @enlarge-text="postFontSize += 0.1"
    ></BlogPost>
  </div>
</template>
```

BlogPost.vue：

```vue
<script setup>
defineProps(['title'])
defineEmits(['enlarge-text'])
</script>

<template>
  <div class="blog-post">
    <h4>{{ title }}</h4>
    <button @click="$emit('enlarge-text')">Enlarge text</button>
  </div>
</template>
```

#### 插槽

将 HTML 元素中的内容传递给组件内部

App.vue：

```vue
<script setup>
import AlertBox from './AlertBox.vue'
</script>

<template>
	<AlertBox>
  	Something bad happened.
	</AlertBox>
</template>
```

BlogPost.vue：

```vue
<template>
  <div class="alert-box">
    <strong>Error!</strong>
    <br/>
    <slot />
  </div>
</template>
```

#### 模板解析注意事项

##### 大小写区分

HTML 标签和属性名称是不分大小写的，所以浏览器会把任何大写的字符解释为小写。这意味着当你使用 DOM 内的模板时，无论是 PascalCase 形式的组件名称、camelCase 形式的 prop 名称还是 v-on 的事件名称，都需要转换为相应等价的 kebab-case (短横线连字符) 形式：

js

```
// JavaScript 中的 camelCase
const BlogPost = {
  props: ['postTitle'],
  emits: ['updatePost'],
  template: `
    <h3>{{ postTitle }}</h3>
  `
}
```

template

```
<!-- HTML 中的 kebab-case -->
<blog-post post-title="hello!" @update-post="onUpdatePost"></blog-post>
```

##### 闭合标签

我们在上面的例子中已经使用过了闭合标签 (self-closing tag)：

template

```
<MyComponent />
```

这是因为 Vue 的模板解析器支持任意标签使用 `/>` 作为标签关闭的标志。

然而在 DOM 内模板中，我们必须显式地写出关闭标签：

template

```
<my-component></my-component>
```

这是由于 HTML 只允许[一小部分特殊的元素](https://html.spec.whatwg.org/multipage/syntax.html#void-elements)省略其关闭标签，最常见的就是 `<input>` 和 `<img>`。对于其他的元素来说，如果你省略了关闭标签，原生的 HTML 解析器会认为开启的标签永远没有结束，用下面这个代码片段举例来说：

template

```
<my-component /> <!-- 我们想要在这里关闭标签... -->
<span>hello</span>
```

将被解析为：

template

```
<my-component>
  <span>hello</span>
</my-component> <!-- 但浏览器会在这里关闭标签 -->
```

##### 元素位置限制

某些 HTML 元素对于放在其中的元素类型有限制，例如 `<ul>`，`<ol>`，`<table>` 和 `<select>`，相应的，某些元素仅在放置于特定元素中时才会显示，例如 `<li>`，`<tr>` 和 `<option>`。

这将导致在使用带有此类限制元素的组件时出现问题。例如：

template

```
<table>
  <blog-post-row></blog-post-row>
</table>
```

自定义的组件 `<blog-post-row>` 将作为无效的内容被忽略，因而在最终呈现的输出中造成错误。我们可以使用特殊的 [`is` attribute](https://cn.vuejs.org/api/built-in-special-attributes.html#is) 作为一种解决方案：

template

```
<table>
  <tr is="vue:blog-post-row"></tr>
</table>
```

#### 动态组件

有些场景会需要在两个组件间来回切换，比如 Tab 界面：

[在演练场中查看示例](https://play.vuejs.org/#eNqNVMGOmzAQ/ZURe2BXCiHbrXpwk1X31mMPvS1V5RiTWAEb2SZNhPLvHdvggLZRE6TIM/P8/N5gpk/e2nZ57HhCkrVhWrQWDLdd+1pI0bRKW/iuGg6VVg2ky9wFDp7G8g9lrIl1H80Bb5rtxfFKMcRzUA+aV3AZQKEEhWRKGgus05pL+5NuYeNwj6mTkT4VckRYujVY63GT17twC6/Fr4YjC3kp5DoPNtEgBpY3bU0txwhgXYojsJoasymSkjeqSHweK9vOWoUbXIC/Y1YpjaDH3wt39hMI6TUUSYSQAz8jArPT5Mj+nmIhC6zpAu1TZlEhmXndbBwpXH5NGL6xWrADMsyaMj1lkAzQ92E7mvYe8nCcM24xZApbL5ECiHCSnP73KyseGnvh6V/XedwS2pVjv3C1ziddxNDYc+2WS9fC8E4qJW1W0UbUZwKGSpMZrkX11dW2SpdcE3huT2BULUp44JxPSpmmpegMgU/tyadbWpZC7jCxwj0v+OfTDdU7ITOrWiTjzTS3Vei8IfB5xHZ4PmqoObMEJHryWXXkuqrVn+xEgHZWYRKbh06uLyv4iQq+oIDnkXSQiwKymlc26n75WNdit78FmLWCMeZL+GKMwlKrhLRcBzhlh51WnSwJPFQr9/zLdIZ007w/O6bR4MQe2bseBJMzer5yzwf8MtzbOzYMkNsOY0+HfoZv1d+lZJGMg8fNqdsfbbio4b77uRVv7I0Li8xxZN1PHWbeHdyTWXc/+zgw/8t/+QsROe9h)

上面的例子是通过 Vue 的 `<component>` 元素和特殊的 `is` attribute 实现的：

template

```
<!-- currentTab 改变时组件也改变 -->
<component :is="tabs[currentTab]"></component>
```

在上面的例子中，被传给 `:is` 的值可以是以下几种：

- 被注册的组件名
- 导入的组件对象

你也可以使用 `is` attribute 来创建一般的 HTML 元素。

当使用 `<component :is="...">` 来在多个组件间作切换时，被切换掉的组件会被卸载。我们可以通过 [`` 组件](https://cn.vuejs.org/guide/built-ins/keep-alive.html)强制被切换掉的组件仍然保持“存活”的状态。

```vue
<script setup>
import Home from './Home.vue'
import Posts from './Posts.vue'
import Archive from './Archive.vue'
import { ref } from 'vue'
 
const currentTab = ref('Home')

const tabs = {
  Home,
  Posts,
  Archive
}
</script>

<template>
  <div class="demo">
    <button
       v-for="(_, tab) in tabs"
       :key="tab"
       :class="['tab-button', { active: currentTab === tab }]"
       @click="currentTab = tab"
     >
      {{ tab }}
    </button>
	  <component :is="tabs[currentTab]" class="tab"></component>
  </div>
</template>
```

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

## 深入组件

### 插槽

#### 默认内容

在外部没有提供任何内容的情况下，可以为插槽指定默认内容。

如果我们想在父组件没有提供任何插槽内容时在 `<button>` 内渲染“Submit”，只需要将“Submit”写在 `<slot>` 标签之间来作为默认内容：

```
<button type="submit">
  <slot>
    Submit <!-- 默认内容 -->
  </slot>
</button>
```

#### 具名插槽

在一个组件中包含多个插槽出口，需要可对插槽命名即可，不命名是default

向 `<BaseLayout>` 传递插槽内容的代码，指令均使用的是缩写形式：

组件外部使用

```vue
<BaseLayout>
  <template #header>
    <h1>Here might be a page title</h1>
  </template>

  <template #default>
    <p>A paragraph for the main content.</p>
    <p>And another one.</p>
  </template>

  <template #footer>
    <p>Here's some contact info</p>
  </template>
</BaseLayout>
```

组件内部：

```vue
<template>
  <div class="container">
    <header>
      <slot name="header"></slot>
    </header>
    <main>
      <slot></slot>
    </main>
    <footer>
      <slot name="footer"></slot>
    </footer>
  </div>
</template>
```

#### 动态插槽名

[动态指令参数](https://cn.vuejs.org/guide/essentials/template-syntax.html#dynamic-arguments)在 `v-slot` 上也是有效的，即可以定义下面这样的动态插槽名：

template

```
<base-layout>
  <template v-slot:[dynamicSlotName]>
    ...
  </template>

  <!-- 缩写为 -->
  <template #[dynamicSlotName]>
    ...
  </template>
</base-layout>
```

注意这里的表达式和动态指令参数受相同的[语法限制](https://cn.vuejs.org/guide/essentials/template-syntax.html#directives)。



#### 作用域插槽

在上面的[渲染作用域](https://cn.vuejs.org/guide/components/slots.html#render-scope)中我们讨论到，插槽的内容无法访问到子组件的状态。

然而在某些场景下插槽的内容可能想要同时使用父组件域内和子组件域内的数据。要做到这一点，我们需要一种方法来让子组件在渲染时将一部分数据提供给插槽。

可以像对组件传递 props 那样，向一个插槽的出口上传递。注意，这里的`v-slot`是作为组件`MyComponent`的属性用于获取子组件数据，而非`template`的属性为其命名。

```vue
<script setup>
import MyComponent from './MyComponent.vue'
</script>

<template>
	<MyComponent v-slot="slotProps">
  	{{ slotProps.text }} {{ slotProps.count }}
  </MyComponent>
</template>
```

组件`MyComponent`内部：

```vue
<script setup>
const greetingMessage = 'hello'
</script>

<template>
  <div>
  	<slot :text="greetingMessage" :count="1"></slot>
	</div>
</template>
```

#### 具名作用域插槽

具名作用域插槽的工作方式也是类似的，插槽 props 可以作为 `v-slot` 指令的值被访问到：`v-slot:name="slotProps"`。当使用缩写时是这样：

template

```
<MyComponent>
  <template #header="headerProps">
    {{ headerProps }}
  </template>

  <template #default="defaultProps">
    {{ defaultProps }}
  </template>

  <template #footer="footerProps">
    {{ footerProps }}
  </template>
</MyComponent>
```

向具名插槽中传入 props：

template

```
<slot name="header" message="hello"></slot>
```



## 组合式函数

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



# Vuex

每一个 Vuex 应用的核心就是 store（仓库）。“store”基本上就是一个容器，它包含着你的应用中大部分的**状态 (state)**。Vuex 和单纯的全局对象有以下两点不同：

1. Vuex 的状态存储是响应式的。当 Vue 组件从 store 中读取状态的时候，若 store 中的状态发生变化，那么相应的组件也会相应地得到高效更新。
2. 你不能直接改变 store 中的状态。改变 store 中的状态的唯一途径就是显式地**提交 (commit) mutation**。这样使得我们可以方便地跟踪每一个状态的变化，从而让我们能够实现一些工具帮助我们更好地了解我们的应用。

提供一个初始 state 对象和一些 mutation：

```js
import { createApp } from 'vue'
import { createStore } from 'vuex'

// 创建一个新的 store 实例
const store = createStore({
  state () {
    return {
      count: 0
    }
  },
  mutations: {
    increment (state) {
      state.count++
    }
  }
})

const app = createApp({ /* 根组件 */ })

// 将 store 实例作为插件安装
app.use(store)
```

现在，你可以通过 `store.state` 来获取状态对象，并通过 `store.commit` 方法触发状态变更：

```
store.commit('increment')

console.log(store.state.count) // -> 1
```

在 Vue 组件中， 可以通过 `this.$store` 访问store实例。现在我们可以从组件的方法提交一个变更：

```
methods: {
  increment() {
    this.$store.commit('increment')
    console.log(this.$store.state.count)
  }
}
```

## State | Vuex

### 单一状态树

Vuex 使用 **单一状态树** ——是的，用一个对象就包含了全部的应用层级状态。 至此它便作为一个“唯一数据源 ( SSOT )”而存在。 这也意味着，每个应用将仅仅包含一个 store 实例。 单一状态树让我们能够直接地定位任一特定的状态片段，在调试的过程中也能轻易地取得整个当前应用状态的快照。 单状态树和模块化并不冲突——在后面的章节里我们会讨论如何将状态和状态变更事件分布到各个子模块中。



### 在 Vue 组件中获得 Vuex 状态

那么我们如何在 Vue 组件中展示状态呢？ 由于 Vuex 的状态存储是响应式的，从 store 实例中读取状态最简单的方法就是在 计算属性 中返回某个状态：

```vue
// 创建一个 Counter 组件
const Counter = {
  template: `<div> { { count }}</div>`,
  computed: {
    count () {
      return store.state.count
    }
  }
}
```

### mapState 辅助函数

当一个组件需要获取多个状态的时候，将这些状态都声明为计算属性会有些重复和冗余。 为了解决这个问题，我们可以使用 mapState 辅助函数帮助我们生成计算属性，让你少按几次键：

```js
// 在单独构建的版本中辅助函数为 Vuex.mapState
import { mapState } from 'vuex'
export default {
  // ...
  computed: mapState({
    // 箭头函数可使代码更简练
    count: state => state.count,
    // 传字符串参数 'count' 等同于 `state => state.count`
    countAlias: 'count',
    // 为了能够使用 `this` 获取局部状态，必须使用常规函数
    countPlusLocalState (state) {
      return state.count + this.localCount
    }
  })
}
```

若不使用则：

```js
export default {
  // ...
  computed: {
    count() {
      return this.$store.state.count;
    },
    countAlias() {
      return this.$store.state.count;
    },
    countPlusLocalState() {
      return this.$store.state.count + this.localCount;
    }
  }
}
```



当映射的计算属性的名称与 state 的子节点名称相同时，我们也可以给 mapState 传一个字符串数组。

```vue
computed: mapState([
  'count'
])
```

### 对象展开运算符

mapState 函数返回的是一个对象。 我们如何将它与局部计算属性混合使用呢？ 通常，我们需要使用一个工具函数将多个对象合并为一个，以使我们可以将最终对象传给 computed 属性。 但是自从有了 对象展开运算符 ，我们可以极大地简化写法：

```vue
computed: {
  localComputed () { /* ... */ },
  // 使用对象展开运算符将此对象混入到外部对象中
  ...mapState({
    // ...
  })
}
```

## Getter | Vuex

有时候我们需要从 store 中的 state 中派生出一些状态，例如对列表进行过滤并计数：

```vue
computed: {
  doneTodosCount () {
    return this.$store.state.todos.filter(todo => todo.done).length
  }
}
```

如果有多个组件需要用到此属性，我们要么复制这个函数，或者抽取到一个共享函数然后在多处导入它——无论哪种方式都不是很理想。

Vuex 允许我们在 store 中定义“getter”（可以认为是 store 的计算属性）。

Getter 接受 state 作为其第一个参数：

```vue
const store = createStore({
  state: {
    todos: [
      { id: 1, text: '...', done: true },
      { id: 2, text: '...', done: false }
    ]
  },
  getters: {
    doneTodos (state) {
      return state.todos.filter(todo => todo.done)
    }
  }
})
```

### 通过属性访问

Getter 会暴露为 store.getters 对象，你可以以属性的形式访问这些值：

```vue
store.getters.doneTodos // -> [ { id: 1, text: '...', done: true }]
```

Getter 也可以接受其他 getter 作为第二个参数：

```vue
getters: {
  // ...
  doneTodosCount (state, getters) {
    return getters.doneTodos.length
  }
}

store.getters.doneTodosCount // -> 1
```

我们可以很容易地在任何组件中使用它：

```vue
computed: {
  doneTodosCount () {
    return this.$store.getters.doneTodosCount
  }
}
```

注意，getter 在通过属性访问时是作为 Vue 的响应式系统的一部分缓存其中的。

### 通过方法访问

你也可以通过让 getter 返回一个函数，来实现给 getter 传参。 在你对 store 里的数组进行查询时非常有用。

```vue
getters: {
  // ...
  getTodoById: (state) => (id) => {
    return state.todos.find(todo => todo.id === id)
  }
}

store.getters.getTodoById(2) // -> { id: 2, text: '...', done: false }
```

注意，getter 在通过方法访问时，每次都会去进行调用，而不会缓存结果。

### mapGetters 辅助函数

mapGetters 辅助函数仅仅是将 store 中的 getter 映射到局部计算属性：

```vue
import { mapGetters } from 'vuex'

export default {
  // ...
  computed: {
    // 使用对象展开运算符将 getter 混入 computed 对象中
    ...mapGetters([
      'doneTodosCount',
      'anotherGetter',
      // ...
    ])
  }
}
```

如果你想将一个 getter 属性另取一个名字，使用对象形式：

```vue
...mapGetters({
  // 把 `this.doneCount` 映射为 `this.$store.getters.doneTodosCount`
  doneCount: 'doneTodosCount'
})
```







## Mutation | Vuex

### Mutation

更改 Vuex 的 store 中的状态的唯一方法是提交 mutation。 Vuex 中的 mutation 非常类似于事件：每个 mutation 都有一个字符串的 事件类型 (type)和一个回调函数 (handler) 。 这个回调函数就是我们实际进行状态更改的地方，并且它会接受 state 作为第一个参数：

```js
const store = createStore({
  state: {
    count: 1
  },
  mutations: {
    increment (state) {
      // 变更状态
      state.count++
    }
  }
})
```

你不能直接调用一个 mutation 处理函数。 这个选项更像是事件注册：“当触发一个类型为 increment 的 mutation 时，调用此函数。 ”要唤醒一个 mutation 处理函数，你需要以相应的 type 调用 store.commit 方法：

```js
store.commit('increment')
```

### 提交载荷（Payload）

你可以向 store.commit 传入额外的参数，即 mutation 的 载荷（payload） ：

```js
// ...
mutations: {
  increment (state, n) {
    state.count += n
  }
}
store.commit('increment', 10)
```

在大多数情况下，载荷应该是一个对象，这样可以包含多个字段并且记录的 mutation 会更易读：

```js
// ...
mutations: {
  increment (state, payload) {
    state.count += payload.amount
  }
}
store.commit('increment', { amount: 10 })
```

### 对象风格的提交方式

提交 mutation 的另一种方式是直接使用包含 type 属性的对象：

```js
store.commit({
  type: 'increment',
  amount: 10
})
```

当使用对象风格的提交方式，整个对象都作为载荷传给 mutation 函数，因此处理函数保持不变：

```js
mutations: {
  increment (state, payload) {
    state.count += payload.amount
  }
}
```

### Mutation 必须是同步函数

一条重要的原则就是要记住 mutation 必须是同步函数 。 为什么？ 请参考下面的例子：

```js
mutations: {
  someMutation (state) {
    api.callAsyncMethod(() => {
      state.count++
    })
  }
}
```

现在想象，我们正在 debug 一个 app 并且观察 devtool 中的 mutation 日志。 每一条 mutation 被记录，devtools 都需要捕捉到前一状态和后一状态的快照。 然而，在上面的例子中 mutation 中的异步函数中的回调让这不可能完成：因为当 mutation 触发的时候，回调函数还没有被调用，devtools 不知道什么时候回调函数实际上被调用——实质上任何在回调函数中进行的状态的改变都是不可追踪的。

### 在组件中提交 Mutation

你可以在组件中使用 this.$store.commit ('xxx') 提交 mutation，或者使用 mapMutations 辅助函数将组件中的 methods 映射为 store.commit 调用（需要在根节点注入 store ）。

```js
import { mapMutations } from 'vuex'

export default {
  // ...
  methods: {
    ...mapMutations([
      'increment', // 将 `this.increment ()` 映射为 `this.$store.commit ('increment')`
      // `mapMutations` 也支持载荷：
      'incrementBy' // 将 `this.incrementBy (amount)` 映射为 `this.$store.commit ('incrementBy', amount)`
    ]),
    ...mapMutations({
      add: 'increment' // 将 `this.add ()` 映射为 `this.$store.commit ('increment')`
    })
  }
}
```



## Action | Vuex

Action 类似于 mutation，不同在于：Action 提交的是 mutation，而不是直接变更状态。Action 可以包含任意异步操作。

### 注册一个简单的 action

```js
const store = createStore({
  state: { count: 0 },
  mutations: { increment (state) { state.count++ } },
  actions: { increment (context) { context.commit('increment') } }
})
```

Action 函数接受一个与 store 实例具有相同方法和属性的 context 对象，因此你可以调用 context.commit 提交一个 mutation，或者通过 context.state 和 context.getters 来获取 state 和 getters。

### 分发 Action

Action 通过 store.dispatch 方法触发：

```js
store.dispatch('increment')
```

我们可以在 action 内部执行 异步 操作：

```js
actions: {
  incrementAsync ({ commit }) {
    setTimeout(() => { commit('increment') }, 1000)
  }
}
```

Actions 支持同样的载荷方式和对象方式进行分发：

```js
store.dispatch('incrementAsync', { amount: 10 })
store.dispatch({ type: 'incrementAsync', amount: 10 })
```

### 在组件中分发 Action

你在组件中使用 this.$store.dispatch ('xxx') 分发 action，或者使用 mapActions 辅助函数将组件的 methods 映射为 store.dispatch 调用（需要先在根节点注入 store ）：

```js
import { mapActions } from 'vuex'
export default {
  // ...
  methods: {
    ...mapActions([ 'increment', 'incrementBy' ]),
    ...mapActions({ add: 'increment' })
  }
}
```

### 组合 Action

Action 通常是异步的，那么如何知道 action 什么时候结束呢？ 更重要的是，我们如何才能组合多个 action，以处理更加复杂的异步流程？

```js
actions: {
  actionA ({ commit }) {
    return new Promise((resolve, reject) => {
      setTimeout(() => { commit('someMutation') resolve() }, 1000)
    })
  }
}
```

现在你可以：

```js
store.dispatch('actionA').then(() => { // ... })
```

在另外一个 action 中也可以：

```js
actions: {
  // ...
  actionB ({ dispatch, commit }) {
    return dispatch('actionA').then(() => { commit('someOtherMutation') })
  }
}
```

如果我们利用 async / await ，我们可以如下组合 action：

```js
// 假设 getData () 和 getOtherData () 返回的是 Promise
actions: {
  async actionA ({ commit }) {
    commit('gotData', await getData())
  },
  async actionB ({ dispatch, commit }) {
    await dispatch('actionA') // 等待 actionA 完成
    commit('gotOtherData', await getOtherData())
  }
}
```

一个 store.dispatch 在不同模块中可以触发多个 action 函数。 在这种情况下，只有当所有触发函数完成后，返回的 Promise 才会执行。



> Searching the web for: `https://vuex.vuejs.org/zh/guide/modules.html markdown format`

[1]: https://vuex.vuejs.org/guide/modules.html
[2]: https://vuex.vuejs.org/guide/
[3]: https://vuex.vuejs.org/api/index.html
[4]: https://vuex.vuejs.org/guide/structure

以下是我从网页[^1^][1]中获取的全部内容，并按照您的要求转化为Markdown格式：

## 模块 | Vuex

由于使用单一状态树，应用的所有状态会集中到一个比较大的对象。然而，当我们的应用在规模上增长时，store可能会变得非常臃肿。为了帮助解决这个问题，Vuex允许我们将store分割成模块。每个模块都可以包含自己的state、mutations、actions、getters，甚至是嵌套的模块——它是自下而上的分形：

```js
const moduleA = {
  state: () => ({ ... }),
  mutations: { ... },
  actions: { ... },
  getters: { ... }
}

const moduleB = {
  state: () => ({ ... }),
  mutations: { ... },
  actions: { ... }
}

const store = createStore({
  modules: {
    a: moduleA,
    b: moduleB
  }
})

store.state.a // -> `moduleA`'s state
store.state.b // -> `moduleB`'s state
```

### 模块的局部状态

在模块的mutations和getters内部，接收的第一个参数将是模块的局部状态。

```js
const moduleA = {
  state: () => ({ count: 0 }),
  mutations: {
    increment (state) {
      // `state` is the local module state
      state.count++
    }
  },
  getters: {
    doubleCount (state) {
      return state.count * 2
    }
  }
}
```

同样，在模块的actions内部，context.state将暴露出局部状态，而根状态将作为context.rootState暴露出来：

```js
const moduleA = {
  // ...
  actions: {
    incrementIfOddOnRootSum ({ state, commit, rootState }) {
      if ((state.count + rootState.count) % 2 === 1) {
        commit('increment')
      }
    }
  }
}
```

此外，在模块的getters内部，根状态将作为第三个参数暴露出来：

```js
const moduleA = {
  // ...
  getters: {
    sumWithRootCount (state, getters, rootState) {
      return state.count + rootState.count
    }
  }
}
```

### 命名空间

默认情况下，actions和mutations仍然会注册在全局命名空间下——这使得多个模块可以对同一种action/mutation类型做出反应。默认情况下，getters也会注册在全局命名空间下。然而，这目前没有实际的功能性目的（它是为了避免破坏性的改变）。你必须小心不要在不同的、非命名空间的模块中定义两个具有相同名称的getters，这会导致错误。

如果你希望你的模块更加自包含或可重用，你可以通过namespaced: true将其标记为命名空间。当模块被注册时，它的所有getters、actions和mutations都将根据模块注册的路径自动命名空间化。例如：

```js
const store = createStore({
  modules: {
    account: {
      namespaced: true,
      // module assets
      state: () => ({ ... }), // module state is already nested and not affected by namespace option
      getters: {
        isAdmin () { ... } // -> getters ['account/isAdmin']
      },
      actions: {
        login () { ... } // -> dispatch ('account/login')
      },
      mutations: {
        login () { ... } // -> commit ('account/login')
      },
      // nested modules
      modules: {
        // inherits the namespace from parent module
        myPage: {
          state: () => ({ ... }),
          getters: {
            profile () { ... } // -> getters ['account/profile']
          }
        },
        // further nest the namespace
        posts: {
          namespaced: true,
          state: () => ({ ... }),
          getters: {
            popular () { ... } // -> getters ['account/posts/popular']
          }
        }
      }
    }
  }
})
```



启用了命名空间的 getter 和 action 会收到局部化的 getter ， dispatch 和 commit 。 换言之，你在使用模块内容（module assets）时不需要在同一模块内额外添加空间名前缀。 更改 namespaced 属性后不需要修改模块内的代码。

### 在带命名空间的模块内访问全局内容（Global Assets）

如果你希望使用全局 state 和 getter， rootState 和 rootGetters 会作为第三和第四参数传入 getter，也会通过 context 对象的属性传入 action。

```js
modules: {
  foo: {
    namespaced: true,
    getters: {
      // 在这个模块的 getter 中，`getters` 被局部化了
      // 你可以使用 getter 的第四个参数来调用 `rootGetters`
      someGetter (state, getters, rootState, rootGetters) {
        getters.someOtherGetter // -> 'foo/someOtherGetter'
        rootGetters.someOtherGetter // -> 'someOtherGetter'
        rootGetters['bar/someOtherGetter'] // -> 'bar/someOtherGetter'
      },
      someOtherGetter: state => { ... }
    },
    actions: {
      // 在这个模块中， dispatch 和 commit 也被局部化了
      // 他们可以接受 `root` 属性以访问根 dispatch 或 commit
      someAction ({ dispatch, commit, getters, rootGetters }) {
        getters.someGetter // -> 'foo/someGetter'
        rootGetters.someGetter // -> 'someGetter'
        rootGetters['bar/someGetter'] // -> 'bar/someGetter'
        dispatch('someOtherAction') // -> 'foo/someOtherAction'
        dispatch('someOtherAction', null, { root: true }) // -> 'someOtherAction'
        commit('someMutation') // -> 'foo/someMutation'
        commit('someMutation', null, { root: true }) // -> 'someMutation'
      },
      someOtherAction (ctx, payload) { ... }
    }
  }
}
```

若需要在全局命名空间内分发 action 或提交 mutation，将 { root: true } 作为第三参数传给 dispatch 或 commit 即可。

### 注册全局 action

如果你希望注册全局 action ，你可添加 root: true，并将这个 action 的定义放在函数 handler 中。例如：

```js
modules: {
  foo: {
    namespaced: true,
    actions: {
      someAction: {
        root: true,
        handler (namespacedContext, payload) { ... } // -> dispatch('someAction')
      },
      someOtherAction (ctx, payload) { ... } // -> dispatch('foo/someOtherAction')
    }
  }
}
```

### 带命名空间的绑定函数

当使用 mapState、mapGetters、mapActions 和 mapMutations 这些函数来绑定带命名空间的模块时，写起来可能比较繁琐：

```js
computed: {
  ...mapState('some/nested/module', {
    a: state => state.a,
    b: state => state.b
  })
},
methods: {
  ...mapActions('some/nested/module', [
    'foo',
    'bar'
  ])
}
```

你可以将模块的空间名称字符串作为第一个参数传递给上述函数，这样所有绑定都会自动将该模块作为上下文。 这样你就可以将上述代码简化为：

```js
computed: {
  ...mapState('some/nested/module', [
    'a',
    'b'
  ])
},
methods: {
  ...mapActions('some/nested/module', [
    'foo',
    'bar'
  ])
}
```

### 模块重用

有时我们可能需要创建一个模块的多个实例，例如：

- 创建多个 store，他们公用同一个模块（例如当 runInNewContext 选项是 false 或 'once'时）
- 在一个 store 中多次注册同一个模块

如果我们使用一个纯对象来声明模块的状态，那么这个状态对象会通过引用被共享，导致每次实例化都会返回同一个状态对象！实际上这就是我们在模块中使用函数声明 state 的原因：

```js
const MyReusableModule = {
  state: () => ({
    foo: 'bar'
  }),
  // mutations, actions, getters...
}
```

