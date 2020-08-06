## Local - Git

### Configuraciones generales

`git config --global user.name "Nombre" ` Establece el nombre con que se registra el autor de los commits realizados
`git config --global user.name "correo@xdxd.com" ` Establece el correo del autor
El par�metro `--global` le indica a git que esa configuraci�n se aplica a todos los repositorios creados en los que no se especifique el user.name y user.email de manera local

`git config --global alias.nombre-alias "comandos a ejecutar" ` Nos permite crear nuestros propios comandos, e.g.

`git config --global alias.lg "log --oneline --decorate --all --graph"`
`git config --global alias.s "status -s -b"` Status silent que muestra las ramas

### Recuperaci�n de archivos
`git checkout -- .` Restaurar los cambios a la versi�n actual

### Ramas
`git checkout -b rama-x` Cambia a la rama especificada en rama-x, el par�metro -b sirve para crearla en el mismo paso, evitando el uso de `git branch rama-x`

`git branch -D rama-x` Forzar la eliminaci�n de la rama, �til cuando se elimina la rama en GitHub y se necesita eliminar la basura del repositorio local

`git diff rama-x master` Compara las diferencias entre ramas

### Tags
`git tag -a v1.0.0 -m "La versi�n MAJOR.MINOR.PATCH"` Seg�n la sem�ntica de versionado semver.org, el par�metro -a indica que es un tag anotado

## Remoto - GitHub

### Interacci�n de archivos entre local y remoto

`git remote add origin url` Configura el repositorio local para comunicarse con el origen determinado por la url del repositorio, cuando el repositorio pertenece a un fork se a�ade tambi�n el repositorio original cambiando origin por upstream

`git push -u origin master` Sube los cambios a GitHub, origin se refiere al repositorio remoto y master a la rama que deseamos subir, el par�metro -u guarda la preferencia de sincronizaci�n para usar posteriormente solo el comando `git push`, cuando se vaya a hacer el push a una rama se ejecuta el comando completo, omitiendo el par�metro -u
`git push :rama-x` Esta acci�n debe ser realizada en el repositorio remoto, �til para eliminar ramas desde la terminal

`git fetch` Compara el repositorio local con el remoto e informa de las diferencias que hay entre ellos

`git pull` Hace un fetch y sincroniza los archivos faltantes del repositorio remoto al local

### Subir tags a GitHub
`git push --tags`