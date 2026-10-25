# Run in Windows PowerShell. Publishes a feature branch and opens a draft PR.
# Never force-pushes, merges, deletes files, or publishes local submissions.
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$expectedBranch = 'codex/color-target-navigation'
$repository = 'tate-no/physical-ai-ros2-gazebo'

function Assert-Exit([string]$action) {
    if ($LASTEXITCODE -ne 0) { throw "$action failed. No further action was taken." }
}

Push-Location -LiteralPath $repoRoot
try {
    $remote = git remote get-url origin
    Assert-Exit 'Read remote'
    if ($remote -notin @("https://github.com/$repository.git", "https://github.com/$repository")) {
        throw 'The remote is not the expected repository.'
    }
    $branch = git branch --show-current
    Assert-Exit 'Read branch'
    if ($branch -ne $expectedBranch) { throw "Expected branch: $expectedBranch" }
    $dirty = git status --porcelain --untracked-files=no
    Assert-Exit 'Check tracked changes'
    if ($dirty) { throw 'Tracked files have changed. Commit or review them before publishing.' }

    gh auth status --hostname github.com
    if ($LASTEXITCODE -ne 0) {
        throw 'Sign in first: gh auth login --hostname github.com --git-protocol https --web'
    }
    gh auth setup-git --hostname github.com
    Assert-Exit 'Set up GitHub authentication'
    git fetch origin
    Assert-Exit 'Fetch remote'
    git merge-base --is-ancestor origin/main HEAD
    if ($LASTEXITCODE -eq 1) {
        git rebase origin/main
        Assert-Exit 'Rebase onto current main (resolve conflicts before retrying)'
    } elseif ($LASTEXITCODE -ne 0) {
        throw 'Could not compare the feature branch with main.'
    }
    git push --set-upstream origin $expectedBranch
    Assert-Exit 'Push feature branch'

    $existing = gh pr list --repo $repository --head $expectedBranch --state open --json url --jq '.[0].url'
    Assert-Exit 'Find existing pull request'
    if ($existing) {
        Write-Output "Draft/review URL: $existing"
    } else {
        gh pr create --repo $repository --draft --base main --head $expectedBranch --title 'Add ordered color-contact navigation with LiDAR recovery' --body-file (Join-Path $PSScriptRoot 'pull_request_body.md')
        Assert-Exit 'Create draft pull request'
    }
    Write-Output 'Published feature branch. main was not merged. Keep the PR URL for review.'
} finally {
    Pop-Location
}
