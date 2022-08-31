# Git

分为工作区和暂存区和已提交区，在工作区的文件不会改变，通过`git add <filename>`会在暂存区添加一份等待提交。



## 初始配置

```bash
git --version  					      #Git版本
git update-git-for-windows		       #升级版本
git config --system [--unset] user.name 用户名    #设置/删除用户签名（全局）
git config --system [--unset] user.email 邮箱     #设置/删除用户签名（全局） 
```



## 基本命令

```bash
git status         #查看本地库状态
git log            #查看每个版本，精简版reflog

# 添加到暂存区
git add 文件名                    #添加至暂存区
git add .                        #添加所有文件至暂存区
git rm --cached hello.txt        # 删除暂存区文件，不影响本地区

# 提交
git commit -m "日志信息" [文件名]  #提交至本地库并附有日志信息（必填）  提交解决冲突时不需要加文件名！
git commit -a -m "日志信息"        #  

# 版本穿梭
git reset --hard commithash      #HEAD、暂存区、工作区（版本穿梭）
```



> 分支冲突：同一个地方有两处不同的修改时触发（仅在一个地方有修改不冲突）

> 在github上请求合并自己的分支是pull request

```bash
# 分支操作
git branch 分支名 [commithash]  #创建分支
git branch [-v]                 #查看分支
git checkout [-b] 分支名        #[-b创建并]切换分支
git merge 分支名                #合并分支
git branch -D/-d name          #(强制)删除分支
```

# Github

### 远程仓库操作

| 命令                               | 作用                                                     |
| :--------------------------------- | :------------------------------------------------------- |
| `git remote add 别名 远程地址`     | 给远程仓库地址起别名                                     |
| `git remote -v`                    | 查看当前所有远程别名                                     |
| `git clone 远程地址`               | 将远程仓库的内容克隆到本地                               |
| `git pull 远程地址别名 远程分支名` | 将远程仓库对于分支最新内容拉下来后与当前本地分支直接合并 |
| `git push 别名（或http地址） 分支` | 推送本地分支上的内容到远程仓库                           |



### 问题

1. `HTTP/2 stream 1 was not closed cleanly before end of the underlying stream`

```bash
git config --global http.version HTTP/1.1
```

2. 更新gitignore文件中的忽略

```bash
git rm -r --cached .
git add .

git commit -m 'update .gitignore'
```

