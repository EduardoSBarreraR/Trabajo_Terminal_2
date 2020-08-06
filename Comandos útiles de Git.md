## Local - Git

### Recuperación de archivos
`git checkout -- .` Restaurar los cambios a la versión actual

### Ramas
`git checkout -b rama-x` Cambia a la rama especificada en rama-x, el parámetro -b sirve para crearla en el mismo paso, evitando el uso de `git branch rama-x`

`git branch -D rama-x` Forzar la eliminación de la rama, útil cuando se elimina la rama en GitHub y se necesita eliminar la basura del repositorio local

`git diff rama-x master` Compara las diferencias entre ramas

### Tags
`git tag -a v1.0.0 -m "La versión MAJOR.MINOR.PATCH"` Según la semántica de versionado semver.org, el parámetro -a indica que es un tag anotado

## Remoto - GitHub

### Interacción de archivos entre local y remoto

`git remote add origin url` Configura el repositorio local para comunicarse con el origen determinado por la url del repositorio, cuando el repositorio pertenece a un fork se añade también el repositorio original cambiando origin por upstream

`git push -u origin master` Sube los cambios a GitHub, origin se refiere al repositorio remoto y master a la rama que deseamos subir, el parámetro -u guarda la preferencia de sincronización para usar posteriormente solo el comando `git push`, cuando se vaya a hacer el push a una rama se ejecuta el comando completo, omitiendo el parámetro -u
`git push :rama-x` Esta acción debe ser realizada en el repositorio remoto, útil para eliminar ramas desde la terminal

`git fetch` Compara el repositorio local con el remoto e informa de las diferencias que hay entre ellos

`git pull` Hace un fetch y sincroniza los archivos faltantes del repositorio remoto al local

### Subir tags a GitHub
`git push --tags`